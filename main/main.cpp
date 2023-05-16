/**
 * @file main.c
 * @author Italo Soares (italo.soares@simova.com.br / italocjs@live.com)
 * @brief Example and development settings. no specific usage intended
 * @version 1.4
 * @date
 * 2023-05-01 22:36:03 - Begin of date keeping.
 * 2023-05-01 22:36:11 - Adding AT commands
 * 2023-05-16 11:39:04 - Main cleaned up, moved functions to separated files
 * 2023-05-16 11:39:16 - code cleanup and released as V1.4
 * @copyright Copyright (c) 2023
 */

int current_baud_rate;
int system_status = -1;    //-1 = error, 0 = ok but not connected, 1 = connected

#include "ota.h"
#include "comms.h"
#include "led.h"

extern "C" void app_main(void)
{
	Serial.begin(115200);    // Serial is always 115200, only used on the programming port and must be initialized fisrt to make sure every ESP_LOGx works
	nvs_flash_init();
	setup_UART();

#ifdef ALWAYS_ENABLE_OTA
	setup_ota();
#endif

	setup_BT();
	setup_bt_workaround();
	setup_command_processor();
	setup_uart2_echo();
	setup_led_task();

	char btname[15];
	snprintf(btname, sizeof(btname), "xBTSAT_%d", getchipID());
	Serial.printf("Firmware: %s | Compile date: %s %s\r\n", FIRMWARE_VERSION, COMPILE_DATE, COMPILE_TIME);
	Serial.printf("Baud Rate is %d, BT SSID: %s, PIN 1234\r\n", current_baud_rate, btname);
	Serial.printf("Bluetooth SSP disabled, WIFI OTA Supported!\r\n");
	Serial.print(AT_CAPABILITIES);
	system_status = 0;
}