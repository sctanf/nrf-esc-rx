#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <math.h>
#include <zephyr/init.h>
#include "battery.h"
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <soc.h>
#include <zephyr/settings/settings.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "battery.h"
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrf.h>
#include <esb.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>
#include <zephyr/init.h>
#include <hal/nrf_gpio.h>
#include <zephyr/sys/poweroff.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>

LOG_MODULE_REGISTER(main, 4);

static struct nvs_fs fs;

#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)

#define STORED_TRACKERS 1
#define RBT_CNT_ID 2
#define STORED_ADDR_0 3

static struct esb_payload rx_payload;
static struct esb_payload tx_payload = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0);
static struct esb_payload tx_payload_pair = ESB_CREATE_PAYLOAD(0, 0, 0, 0, 0, 0, 0, 0, 0);

// this was randomly generated
uint8_t discovery_base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
uint8_t discovery_base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
uint8_t discovery_addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6};
uint8_t base_addr_0[4] = {0,0,0,0};
uint8_t base_addr_1[4] = {0,0,0,0};
uint8_t addr_prefix[8] = {0,0,0,0,0,0,0,0};
uint8_t stored_trackers = 0;
uint64_t stored_tracker_addr[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t pairing_buf[8] = {0,0,0,0,0,0,0,0};

uint8_t reset_mode = -1;

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)
const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, led_gpios);
static const struct pwm_dt_spec servo = PWM_DT_SPEC_GET(DT_NODELABEL(servo));
const struct gpio_dt_spec esc = GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, esc_gpios);

int tickrate = 5;

uint8_t batt;
uint8_t batt_v;
uint32_t batt_pptt;

bool main_running = false;

unsigned int last_batt_pptt[16] = {10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001,10001};
int8_t last_batt_pptt_i = 0;
bool system_off_main = false;
bool send_data = false;
int16_t pot_val = 0;
int16_t pot_val_actual = 0;
int64_t idle_start_time = 0;
int64_t idle_end_time = 0;
int64_t last_data_sent = 0;
bool idle_esc = true;
#define esc_idle_time_dur 60000
#define esc_start_time_dur 800

bool throttle = false;

void event_handler(struct esb_evt const *event)
{
	switch (event->evt_id)
	{
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		break;
	case ESB_EVENT_RX_RECEIVED:
		if (esb_read_rx_payload(&rx_payload) == 0) {
			if (rx_payload.length == 8) {
				for (int i = 0; i < 8; i++) {
					pairing_buf[i] = rx_payload.data[i];
				}
				esb_write_payload(&tx_payload_pair); // Add to TX buffer
			} else if (rx_payload.length == 10) {
				if (rx_payload.data[0] != rx_payload.data[2]) break;
				if (rx_payload.data[1] != rx_payload.data[3]) break;
				pot_val = (int16_t)rx_payload.data[0] << 8 | rx_payload.data[1];
				last_data_sent = k_uptime_get();
				esb_write_payload(&tx_payload); // Add to TX buffer
			}
		}
		break;
	}
}

int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	sys_notify_init_spinwait(&clk_cli.notify);
	err = onoff_request(clk_mgr, &clk_cli);
	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
	} while (err);
	return 0;
}

int esb_initialize_tx(void)
{
	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	// config.mode = ESB_MODE_PTX;
	config.event_handler = event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	config.retransmit_count = 0;
	config.tx_mode = ESB_TXMODE_MANUAL;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	esb_init(&config);
	esb_set_base_address_0(base_addr_0);
	esb_set_base_address_1(base_addr_1);
	esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	return 0;
}

int esb_initialize(void)
{
	struct esb_config config = ESB_DEFAULT_CONFIG;

	// config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = ESB_MODE_PRX;
	config.event_handler = event_handler;
	config.bitrate = ESB_BITRATE_1MBPS;
	// config.crc = ESB_CRC_16BIT;
	config.tx_output_power = 8;
	// config.retransmit_delay = 600;
	// config.retransmit_count = 3;
	// config.tx_mode = ESB_TXMODE_AUTO;
	// config.payload_length = 32;
	config.selective_auto_ack = true;

	// Fast startup mode
	NRF_RADIO->MODECNF0 |= RADIO_MODECNF0_RU_Fast << RADIO_MODECNF0_RU_Pos;
	// nrf_radio_modecnf0_set(NRF_RADIO, true, 0);

	esb_init(&config);
	esb_set_base_address_0(base_addr_0);
	esb_set_base_address_1(base_addr_1);
	esb_set_prefixes(addr_prefix, ARRAY_SIZE(addr_prefix));

	return 0;
}

void configure_system_off(void){
	// Configure dock interrupt
	// Set system off
	sys_poweroff();
}

void power_check(void) {
	int batt_mV;
	uint32_t batt_pptt = read_batt_mV(&batt_mV);
	if (batt_pptt == 0) {
		gpio_pin_set_dt(&led, 0); // Turn off LED
		//configure_system_off();
	}
	LOG_INF("Battery %u%% (%dmV)", batt_pptt/100, batt_mV);
}

void wait_for_threads(void) {
	while (main_running) {
		k_usleep(1);
	}
}

int main(void)
{
	int32_t reset_reason = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = NRF_POWER->RESETREAS; // Clear RESETREAS
	uint8_t reboot_counter = 0;

	//power_check(); // check the battery first before continuing (4ms delta to read from ADC)

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_set_dt(&led, 1);

	gpio_pin_configure_dt(&esc, GPIO_OUTPUT);

	struct flash_pages_info info;
	fs.flash_device = NVS_PARTITION_DEVICE;
	fs.offset = NVS_PARTITION_OFFSET; // Start NVS FS here
	flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
	fs.sector_size = info.size; // Sector size equal to page size
	fs.sector_count = 4U; // 4 sectors
	nvs_mount(&fs);

	if (reset_reason & 0x01) { // Count pin resets
		nvs_read(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		reset_mode = reboot_counter;
		reboot_counter++;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
		k_msleep(1000); // Wait before clearing counter and continuing
		reboot_counter = 0;
		nvs_write(&fs, RBT_CNT_ID, &reboot_counter, sizeof(reboot_counter));
	}
	gpio_pin_set_dt(&led, 0);

	if (reset_mode == 1) { // Clear stored data
		nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		for (int i = 0; i < 16; i++) {
			nvs_write(&fs, STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		}
		LOG_INF("NVS Reset");
		reset_mode = 0; // Clear reset mode
	} else {
		nvs_read(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
		for (int i = 0; i < 16; i++) {
			nvs_read(&fs, STORED_ADDR_0+i, &stored_tracker_addr[i], sizeof(stored_tracker_addr[0]));
		}
		LOG_INF("%d devices stored", stored_trackers);
	}

	// TODO add back separate pairing / clear pairing

	clocks_start();

	if (stored_trackers == 0) { // Pairing mode
		LOG_INF("Starting in pairing mode");
		for (int i = 0; i < 4; i++) {
			base_addr_0[i] = discovery_base_addr_0[i];
			base_addr_1[i] = discovery_base_addr_1[i];
		}
		for (int i = 0; i < 8; i++) {
			addr_prefix[i] = discovery_addr_prefix[i];
		}
		esb_initialize();
		esb_start_rx();
		tx_payload_pair.noack = false;
		uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
		LOG_INF("Device address %lld", addr);
		for (int i = 0; i < 6; i++) {
			tx_payload_pair.data[i+2] = (addr >> (8 * i)) & 0xFF;
		}
		int blink = 0;
		while (true) { // Run indefinitely (User must reset/unplug dongle)
			uint64_t found_addr = 0;
			for (int i = 0; i < 6; i++) { // Take device address from RX buffer
				found_addr |= (uint64_t)pairing_buf[i+2] << (8*i);
			}
			uint16_t send_tracker_id = stored_trackers; // Use new tracker id
			for (int i = 0; i < stored_trackers; i++) { // Check if the device is already stored
				if (found_addr != 0 && stored_tracker_addr[i] == found_addr) {
					//LOG_INF("Found device linked to id %d with address %lld", i, found_addr);
					send_tracker_id = i;
				}
			}
			if (found_addr != 0 && send_tracker_id == stored_trackers && stored_trackers < 16) { // New device, add to NVS
				LOG_INF("Added device on id %d with address %lld", stored_trackers, found_addr);
				stored_tracker_addr[stored_trackers] = found_addr;
				nvs_write(&fs, STORED_ADDR_0+stored_trackers, &stored_tracker_addr[stored_trackers], sizeof(stored_tracker_addr[0]));
				stored_trackers++;
				nvs_write(&fs, STORED_TRACKERS, &stored_trackers, sizeof(stored_trackers));
			}
			if (send_tracker_id < 16) { // Make sure the dongle is not full
				tx_payload_pair.data[0] = pairing_buf[0]; // Use int sent from device to make sure packet is for that device
			} else {
				tx_payload_pair.data[0] = 0; // Invalidate packet
			}
			tx_payload_pair.data[1] = send_tracker_id; // Add tracker id to packet
			//esb_flush_rx();
			//esb_flush_tx();
			//esb_write_payload(&tx_payload_pair); // Add to TX buffer
			if (blink == 0) {
				gpio_pin_set_dt(&led, 1);
				k_msleep(100);
				gpio_pin_set_dt(&led, 0);
				k_msleep(400);
			}
			else
				k_msleep(500);
			blink++;
			blink %= 2;
		}
	}

	// Generate addresses from device address
	uint64_t addr = (((uint64_t)(NRF_FICR->DEVICEADDR[1]) << 32) | NRF_FICR->DEVICEADDR[0]) & 0xFFFFFF;
	uint8_t buf[8] = {0,0,0,0,0,0,0,0};
	for (int i = 0; i < 6; i++) {
		buf[i+2] = (addr >> (8 * i)) & 0xFF;
	}
	uint8_t buf2[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	for (int i = 0; i < 4; i++) {
		buf2[i] = buf[i+2];
		buf2[i+4] = buf[i+2] + buf[6];
	}
	for (int i = 0; i < 8; i++) {
		buf2[i+8] = buf[7] + i;
	}
	for (int i = 0; i < 16; i++) {
		if (buf2[i] == 0x00 || buf2[i] == 0x55 || buf2[i] == 0xAA) {
			buf2[i] += 8;
		};
	}
	for (int i = 0; i < 4; i++) {
		base_addr_0[i] = buf2[i];
		base_addr_1[i] = buf2[i+4];
	}
	for (int i = 0; i < 8; i++) {
		addr_prefix[i] = buf2[i+8];
	}

	esb_initialize();
	esb_start_rx();

	tx_payload.noack = false;

	while (1)
	{
		// Get start time
		int64_t time_begin = k_uptime_get();

		int batt_mV;
		batt_pptt = read_batt_mV(&batt_mV);

		last_batt_pptt[last_batt_pptt_i] = batt_pptt;
		last_batt_pptt_i++;
		last_batt_pptt_i %= 15;
		for (uint8_t i = 0; i < 15; i++) {  // Average battery readings across 16 samples
			if (last_batt_pptt[i] == 10001) {
				batt_pptt += batt_pptt / (i + 1);
			} else {
				batt_pptt += last_batt_pptt[i];
			}
		}
		batt_pptt /= 16;
		if (batt_pptt + 49 < last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt + 49;} // Lower bound -100pptt
		else if (batt_pptt - 49 > last_batt_pptt[15]) {last_batt_pptt[15] = batt_pptt - 49;} // Upper bound +0pptt
		else {batt_pptt = last_batt_pptt[15];} // Effectively 100-10000 -> 1-100%

		// format for packet send
		batt = batt_pptt / 100;
//		if (batt < 1) {batt = 1;} // Clamp to 1%
		batt_mV /= 10;
		batt_mV -= 245;
		if (batt_mV < 0) {batt_v = 0;} // Very dead but it is what it is
		else if (batt_mV > 255) {batt_v = 255;}
		else {batt_v = batt_mV;} // 0-255 -> 2.45-5.00V
		tx_payload.data[0] = batt;
		tx_payload.data[1] = batt;

		if (k_uptime_get() > last_data_sent + 500) pot_val = 0;

		float change_sec = 2.0;
		float max_change = change_sec * 32768.0 * tickrate / 1000.0;
		float idle_val_f = 0.08;
		float idle_val = idle_val_f * 32768.0;
		int16_t throttle_val = 0.88 * 32768.0;
		if (pot_val > -idle_val && pot_val < idle_val)
		{
			pot_val = 0; // actual should be 0
			if (pot_val_actual < -idle_val || pot_val_actual > idle_val)
			{
				idle_start_time = k_uptime_get();
			}
			else if (k_uptime_get() > idle_start_time + esc_idle_time_dur)
			{
				idle_start_time = 0;
				idle_esc = true;
				// actual should be off
			}
		}
		else if (idle_esc) {
			idle_end_time = k_uptime_get();
			idle_esc = false;
		}

		if ((batt < 30) && (throttle == false)) throttle = true;
		if ((batt > 60) && (throttle == true)) throttle = false;
		if (throttle) {
			if (pot_val > throttle_val) pot_val = throttle_val;
			if (pot_val < -throttle_val) pot_val = -throttle_val;
		}
		
		int16_t pot_val_actual2 = pot_val;
		if (k_uptime_get() < idle_end_time + esc_start_time_dur)
		{
			pot_val_actual2 = 0; // actual should be 0
		}

		float max_fwd_f = 0.81;
		float mav_rev_f = 0.86;
		if (pot_val_actual2 > idle_val) pot_val_actual2 = idle_val + ((float)pot_val_actual2 - idle_val) * ((max_fwd_f - idle_val_f) / (1 - idle_val_f));
		if (pot_val_actual2 < -idle_val) pot_val_actual2 = -idle_val - ((float)(-pot_val_actual2) - idle_val) * ((mav_rev_f - idle_val_f) / (1 - idle_val_f));
		if (pot_val_actual2 > max_fwd_f * 32768) pot_val_actual2 = max_fwd_f * 32768;
		if (pot_val_actual2 < -mav_rev_f * 32768) pot_val_actual2 = -mav_rev_f * 32768;
		if (pot_val_actual2 < pot_val_actual - max_change) pot_val_actual -= max_change;
		else if (pot_val_actual2 > pot_val_actual + max_change) pot_val_actual += max_change;
		else pot_val_actual = pot_val_actual2;

		float pot_val_f = (float)pot_val_actual / 32768;
		pot_val_f += 1;
		pot_val_f *= 500;
		pot_val_f += 1000;
		pot_val_f *= 1000;

		if (idle_esc) gpio_pin_set_dt(&led, 0);
		else gpio_pin_set_dt(&led, 1);

		if (idle_esc) gpio_pin_set_dt(&esc, 0);
		else gpio_pin_set_dt(&esc, 1);

		if (idle_esc) pwm_set_pulse_dt(&servo, 0);
		else pwm_set_pulse_dt(&servo, (uint32_t)pot_val_f);

		// Get time elapsed and sleep/yield until next tick
		int64_t time_delta = k_uptime_get() - time_begin;
		if (time_delta > tickrate)
		{
			k_yield();
		}
		else
		{
			k_msleep(tickrate - time_delta);
		}
	}
}
// Receiver, receives pwm and transmits battery and rpm and data to controller