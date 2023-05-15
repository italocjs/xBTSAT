/**
 * @file config.h
 * @author Italo Soares (italocjs@live.com)
 * @brief 
 * @version 0.1
 * @date 2023-05-15
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#pragma once

#define FIRMWARE_VERSION "xbtsat_v1_4_4_RC"
#define COMPILE_DATE __DATE__
#define COMPILE_TIME __TIME__

#define DEFAULT_UART2_BAUD_RATE 9600    // if the NVS is clean, which
#define COMMAND_PROCESSOR_BUFFER_SIZE 10    // how many messages can be stored in the buffer for processing. 10 is already overkill
#define COMMAND_PROCESSOR_SPEED_MS 1        // How fast will the command_processor read the buffer
#define ENABLE_AT_PROTOCOL_SUPPORT
#define DEBUG_COMMAND_PROCESSOR

//#define ALWAYS_ENABLE_OTA // if defined OTA will be started on boot, if not, only by calling AT+OTA
#define REQUIRE_LOGIN
#define PAGE_USERNAME "simova"
#define PAGE_PSWD "simova"
const char *host = "esp32";
const char *ssid;    // = "xBTSAT_OTA";
const char *password = "12345678";

#define LED_GPIO GPIO_NUM_23