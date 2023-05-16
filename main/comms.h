/**
 * @file comms.h
 * @author Italo Soares (italocjs@live.com)
 * @brief
 * @version 1.4
 * @date 2023-05-15
 * 2023-05-16 11:34:22 code cleanup and released as V1.4
 * @copyright Copyright (c) 2023
 */

#pragma once
#include "config.h"
extern int current_baud_rate;
extern int system_status;

#include "Arduino.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <Preferences.h>
#include <nvs_flash.h>
#include "esp_log.h"
#include "BluetoothSerial.h" 
Preferences preferences;
BluetoothSerial SerialBT;
TaskHandle_t Handle_task_cmd_processor = NULL;
QueueHandle_t queue_uart;
QueueHandle_t queue_bluetooth;

String AT_CAPABILITIES =
    "AT+? - Exibe ajuda\r\n"
    "AT+HELP - Exibe ajuda\r\n"
    "AT+BAUD - Responde a velocidade atual\r\n"
    "AT+BAUD=9600 - Define a nova velocidade\r\n"
    "AT+VER - Responde a versao de firmware\r\n"
    "AT+RESET - Restaura todas as configuracoes\r\n"
    "AT+OTA - Ativa o Wifi para atualizacao OTA\r\n";

void command_ATE_BAUD(String data)
{
	ESP_LOGI("", "RUNNING AT+BAUD=");
	int start_pos = data.indexOf('=');
	int new_baud_rate = (data.substring(start_pos + 1)).toInt();

	if (new_baud_rate < 100 || new_baud_rate > 250000)
	{
		// SerialBT.print("AT+BAUD= ERROR.  Value must be between 100 bps and 250000bps");
		SerialBT.print("AT+BAUD= ERROR.  Valor deve ser entre 100 e 250000bps");
		return;
	}
	current_baud_rate = new_baud_rate;
	Serial2.end();
	Serial2.begin(current_baud_rate);

	if (preferences.begin("SYSTEM", false) == true)
	{
		preferences.putInt("uart_speed", new_baud_rate);
		current_baud_rate = preferences.getInt("uart_speed", new_baud_rate);
		preferences.end();
		// SerialBT.printf("AT+OK - new baud is %d, Saved to memory", current_baud_rate);
		SerialBT.printf("AT+OK - Nova velocidade: %d, Salvo na memoria", current_baud_rate);
		ESP_LOGI("", "AT+OK - new baud is %d, Saved to memory", current_baud_rate);
	}
	else
	{
		SerialBT.printf("AT+FAILED - Nova velocidade: %d, Falha ao salvar na memoria", current_baud_rate);
		ESP_LOGE("", "AT+FAILED - new baud is %d, but failed to save to memory", current_baud_rate);
	}
}

TaskHandle_t Handle_RestartHelper;
// Auxiliary task to reboot ESP32, its a workaround to allow log messages to run before the ESP kills itself
void TASK_RestartHelper(void *pvParameters)
{
	ESP_LOGW("", "ESP WILL REBOOT IN 5 SECONDS");
	vTaskDelay(5000 / portTICK_PERIOD_MS);
	while (1)
	{
		vTaskDelay(10 / portTICK_PERIOD_MS);
		ESP.restart();
	}
}

void command_AT_RESET()
{
	ESP_LOGI("", "RUNNING AT+RESET");
	nvs_flash_erase();
	SerialBT.println("AT+OK - Reiniciando para os padroes de fabrica em 5 segundos");
	xTaskCreatePinnedToCore(TASK_RestartHelper,    /* Task function. */
	                        "TASK_RestartHelper",  /* name of task. */
	                        2400,                  /* Stack size of task */
	                        NULL,                  /* parameter of the task */
	                        1,                     /* priority of the task */
	                        &Handle_RestartHelper, /* Task handle to keep track of created task */
	                        1);
}

/**
 * @brief Check a given string for any AT commands, if valid then process the command.
 *
 * @param inData
 * @param output_array //i guess will be useless
 * @param output_array_size //i guess will be useless
 * @param source cmd_src_t , Where is the data coming from? (useful for responding)
 * @return true if any command was processed, false if no correspondence was found
 */
bool process_at_protocol(char *input_buffer)
{
	String inData = "";
	inData = input_buffer;
	inData.toUpperCase();               // makes sure that even if the command was typed in lower-case it will be processed

	if (inData.indexOf("AT+") == -1)    // if command is NOT an AT
	{
		return false;
	}
	/* #region System protocol (AT+COMMANDS) */
	else if (inData.indexOf("AT+BAUD=") != -1)
	{
		ESP_LOGI("", "AT+BAUD=");
		command_ATE_BAUD(inData);
		return true;
	}
	else if (inData.indexOf("AT+BAUD") != -1)
	{
		SerialBT.printf("AT+OK - Velocidade atual: %d", current_baud_rate);
		return true;
	}
	else if (inData.indexOf("AT+RESET") != -1)
	{
		ESP_LOGI("", "AT+RESET");
		command_AT_RESET();
		return true;
	}
	else if (inData.indexOf("AT+VER") != -1)
	{
		SerialBT.printf("AT+OK - Firwmare atual: %s", FIRMWARE_VERSION);
		return true;
	}
	else if (inData.indexOf("AT+OTA") != -1)
	{
		ESP_LOGI("", "AT+OTA");
		setup_ota();
		SerialBT.print("AT+OTA - Wifi ativado, conecte-se a 192.128.1.1 para atualizar o firmware\r\n");
		return true;
	}
	else if ((inData.indexOf("AT+?") != -1) || (inData.indexOf("AT+HELP") != -1))
	{
		SerialBT.print(AT_CAPABILITIES);
		return true;
	}
	else    // Was an AT, but unknown
	{
		SerialBT.print("AT+ERROR Unknown command");
		SerialBT.print(AT_CAPABILITIES);
	}
	/* #endregion */
	return false;    // If no command match was found, return false
}

void task_cmd_processor(void *arg)
{
	char rxbuff[256];
	while (1)
	{
		// ========================= GET THE BUFFER DATA =================================
		if (xQueueReceive(queue_bluetooth, &(rxbuff), (TickType_t)pdMS_TO_TICKS(COMMAND_PROCESSOR_SPEED_MS)))
		{
#ifdef DEBUG_COMMAND_PROCESSOR
			ESP_LOGI("", "Incoming queue_bluetooth ===  %s", rxbuff);
#endif

			// ========================= TRY TO PROCESS COMMANDS =================================
#ifdef ENABLE_AT_PROTOCOL_SUPPORT
			if (process_at_protocol(rxbuff) == true)
			{
#ifdef DEBUG_COMMAND_PROCESSOR
				ESP_LOGI("", "AT command processed");
#endif
			}
			else
			{
#ifdef DEBUG_COMMAND_PROCESSOR
				ESP_LOGI("", "Not AT command, forwarding");
#endif
				Serial2.print(rxbuff);
			}
#endif
		}
	}
	vTaskDelay(pdMS_TO_TICKS(COMMAND_PROCESSOR_SPEED_MS));
}

/**
 * @brief This callback will get any data coming from the bluetooth and put it into the command_processor queue.
 * this has to be as fast as possible to avoid bluetooth crash, so no processing, comparing or whatever here.
 * NOTE: This crashing issue was hard as fuck to diagnose, sometimes it frozen without reboot, sometimes crashed with
 * freertos erros (not helpful) so do NOT process anything more than absolutely needed here.
 * @param buffer
 * @param size
 */
void callback_ESP_BT(const uint8_t *buffer, size_t size)
{
	// String inData;
	// for (size_t i = 0; i < size; i++)
	// {
	// 	inData += char(buffer[i]);
	// }
	// inData.replace("\r", "");
	// inData.replace("\n", "");
	char rxbuff[256];
	snprintf(rxbuff, size + 1, "%s", (char *)buffer);    // Without the +1 it was missing CRLF

#ifdef DEBUG_COMMAND_PROCESSOR
	ESP_LOGI("", "Sending to queue_bluetooth: %s", rxbuff);
	// ESP_LOGI("", "Sending to queue_bluetooth: %s", inData.c_str());
#endif

	xQueueSend(queue_bluetooth, rxbuff, (TickType_t)pdMS_TO_TICKS(COMMAND_PROCESSOR_SPEED_MS));
	// xQueueSend(queue_bluetooth, (void*)inData.c_str(), (TickType_t)10);
}

void setup_BT()
{
	char btname[15];
	snprintf(btname, sizeof(btname), "xBTSAT_%d", getchipID());
	SerialBT.onData(callback_ESP_BT);
	SerialBT.setTimeout(2000);
	SerialBT.begin(btname);    // Bluetooth device name
	                           // SerialBT.enableSSP();
	char _buf[60];
	snprintf(_buf, sizeof(_buf), "[I] Bluetooth SPP started, SSID %s", btname);
	ESP_LOGI("", "%s", _buf);
}

void setup_command_processor()
{
	char txbuff[256];
	queue_bluetooth = xQueueCreate(COMMAND_PROCESSOR_BUFFER_SIZE, sizeof(txbuff));
	if (queue_bluetooth == NULL)
	{
		ESP_LOGE("", "Command_processor not initialized - Unable to create queue_bluetooth");
		return;
	}
	xTaskCreatePinnedToCore(task_cmd_processor, "task_cmd_processor", 4096, NULL, 10, &Handle_task_cmd_processor, 1);
	ESP_LOGI("", "Command_processor initialized");
}

bool setup_UART()
{
	if (preferences.begin("SYSTEM", false) == false)
	{
		ESP_LOGE("", "Failed to begin system nvs");
	}
	if (preferences.isKey("uart_speed") == false)
	{
		ESP_LOGI("", "key does not exist, creating");
		preferences.putInt("uart_speed", DEFAULT_UART2_BAUD_RATE);
		current_baud_rate = preferences.getInt("uart_speed", DEFAULT_UART2_BAUD_RATE);
		ESP_LOGI("", "Created on nvs: uart_speed = %d", current_baud_rate);
	}
	else
	{
		current_baud_rate = preferences.getInt("uart_speed", DEFAULT_UART2_BAUD_RATE);
		ESP_LOGI("", "Read from nvs: uart_speed = %d", current_baud_rate);
	}
	Serial2.begin(current_baud_rate);
	preferences.end();
	return true;
}

TaskHandle_t Handle_BT_RESET_HELPER;
// Auxiliary task to reboot ESP32, its a workaround to allow log messages to run before the ESP kills itself
void TASK_BT_RESET_HELPER(void *pvParameters)
{
	ESP_LOGW("", "started");
	bool previous_state = false;
	while (1)
	{
		bool current_state = SerialBT.hasClient();
		if (current_state == true)
		{
			system_status = 1;
			previous_state = true;
		}
		else if (current_state == false && previous_state == true)
		{
			system_status = 0;
			previous_state = current_state;
			ESP_LOGI("", "CLIENT DISCONNECTED, RESTARTING THE BLUETOOTH STACK");
			SerialBT.end();
			vTaskDelay(500 / portTICK_PERIOD_MS);
			char btname[15];
			snprintf(btname, sizeof(btname), "xBTSAT_%d", getchipID());
			SerialBT.onData(callback_ESP_BT);
			SerialBT.begin(btname);    // Bluetooth device name
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}

void setup_bt_workaround()
{
	xTaskCreatePinnedToCore(TASK_BT_RESET_HELPER,    /* Task function. */
	                        "TASK_BT_RESET_HELPER",  /* name of task. */
	                        2400,                    /* Stack size of task */
	                        NULL,                    /* parameter of the task */
	                        1,                       /* priority of the task */
	                        &Handle_BT_RESET_HELPER, /* Task handle to keep track of created task */
	                        1);
}

bool read_messages(String &inData, int timeout)
{
	bool TIMEOUT_EXPIRED = false;
	bool message_completed = false;
	int time = 0;
	while (!(message_completed == true || TIMEOUT_EXPIRED == true))    // Check if there is something in the serial buffer
	{
		char recieved;
		if (Serial2.available() > 0)
		{
			recieved = Serial2.read();
			inData += recieved;
			if (recieved == '\n')
			{
				message_completed = true;
			}
		}

		else if (time > timeout)
		{
			TIMEOUT_EXPIRED = true;
			ESP_LOGW("", "Receiving timeout");
			return false;
		}
		else
		{
			time++;
			delay(1);    // Nesse caso é melhor usar o delay e travar o processador, evita mensagem corrompida.
		}
	}
	// inData.replace("\r", "");
	// inData.replace("\n", "");
	return true;
}

static void echo_task(void *arg)
{
	while (1)
	{
		int len = Serial2.available();
		if (len > 0)
		{
			String inData;
			for (size_t i = 0; i < len; i++)
			{
				char recieved = Serial2.read();
				inData += recieved;
			}
			// read_messages(inData, 50);
			//			inData = Serial2.readString();
			// Serial.print(inData);
			SerialBT.print(inData);
		}
		vTaskDelay(5 / portTICK_PERIOD_MS);
	}
}

void setup_uart2_echo() { xTaskCreate(echo_task, "uart_echo_task", 2048, NULL, 10, NULL); }
