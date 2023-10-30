/**
 * @file config.h
 * @author Italo Soares (italocjs@live.com)
 * @brief
 * @version 1.4
 * @date 2023-05-15
 * 2023-05-16 11:30:08 code cleanup and released as V1.4
 * @copyright Copyright (c) 2023
 */
#pragma once

// FIRMWARE VERSION
#define FIRMWARE_VERSION "xbtsat_v1_5_0"
#define COMPILE_DATE __DATE__
#define COMPILE_TIME __TIME__

// MAIN SETTINGS
#define DEFAULT_UART2_BAUD_RATE 9600        // used if not modified by AT+BAUD= or not found in NVS
#define COMMAND_PROCESSOR_BUFFER_SIZE 10    // how many messages can be stored in the buffer for processing. 10 is overkill
#define COMMAND_PROCESSOR_SPEED_MS 1        // How fast will the command_processor read the buffer
#define ENABLE_AT_PROTOCOL_SUPPORT
//#define DEBUG_COMMAND_PROCESSOR             // Listen to incoming messages on Serial0

// WIFI OTA SUPPORT
//#define REQUIRE_LOGIN
#define PAGE_USERNAME "simova"
#define PAGE_PSWD "simova"
// #define ALWAYS_ENABLE_OTA // if defined OTA will be started on boot, if not, only by calling AT+OTA
const char *host = "simova";
const char *ssid;    // xBTSAT_xxxxxx (HEX for UUID)
const char *password = "12345678";

#define LED_GPIO GPIO_NUM_23    // This is the indicator LED