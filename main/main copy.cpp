/**
 * @file main.c
 * @author Italo Soares (italo.soares@simova.com.br / italocjs@live.com)
 * @brief Example and development settings. no specific usage intended
 * @version 1.2
 * @date
 * 2023-05-01 22:36:03 - Begin of date keeping.
 * 2023-05-01 22:36:11 - Adding AT commands
 *
 * @copyright Copyright (c) 2023
 * tasks: echo_task
 */

// #include "command_processor.h"
// extern QueueHandle_t queue_bluetooth;    // Needed to send data to command_processor.h
// extern QueueHandle_t queue_uart;    // Needed to send data to command_processor.h

/**
 * @brief
 * Baseado no exemplo acceptor do ESP IDF.
 *
 * Funcionamento:
 * -Cria o monstro bluetooth e seus callbacks
 * -Seta o nome baseado no MAC ADDRESS
 * -Inicia o Serial2 em RX 16  -  TX 17
 * -Faz a transcrição do que passa no Serial para o Bluetooth e vice versa,
 * -Printa tudo no Serial1
 *
 *
 * Como essa bagunça funciona:
 * O SPP só funfa atraves do callback, tudo que for relacionado a gerenciamento de conexao e sessão deve ser feito
 * nos cases desse calback.
 *
 * o SPP_WRITE foi chatinho de descobrir como usar o handle, veja implementacao seguindo _spp_client_handle
 * Está usando RX2 e TX2 nos pinos padrão (16 e 17).
 * Está usando o LED no pino do xBTSAT (23)
 * utiliza a int system_status para trocar o status do bluetooth, se algum setup falhar ele vira -1,
 *  se tudo der certo vira 0, e quando alguem conectar no SPP vira 1.
 *
 *
 *
 * Por algum motivo se mudar de pasta caga o menuconfig,  a config mais importante são:
 *      Secure Simple Pairing DESMARCADO
 *      Use dynamic memory allocation in BT/BLE stack MARCADO
 *      BR/EDR ACL Max Connections = 1
 *
 *
 * nome fica em ESP_SPP_START_EVT
 *
 */

// #region Includes e defines

#include <esp_system.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <freertos/queue.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "sys/time.h"

#include "time.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <string>

#define SPP_TAG "ITALO_SPP_DEBUG"
#define ITALO_TAG "ITALO_OTHER_DEBUG "
#define CHIP_ID_TAG "CHIP_ID() "
#define SPP_SERVER_NAME "SPP_SERVER"
#define EXAMPLE_DEVICE_NAME "xBTSAT_DEV_v200"    // Em teoria vai sumir daqui pq vai pegar pela uuid
#define SPP_SHOW_DATA 1
#define SPP_SHOW_SPEED 0
#define SPP_SHOW_MODE SPP_SHOW_DATA /*Choose show mode: show data or speed*/

#define ECHO_TEST_TXD (17)
#define ECHO_TEST_RXD (16)
#define ECHO_TEST_RTS (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS (UART_PIN_NO_CHANGE)

#define UART_NUM (2)
#define ECHO_UART_BAUD_RATE (9600)
#define ECHO_TASK_STACK_SIZE (2048)
static const char *TAG = "UART TEST";

#define BUF_SIZE (1024)

#define COMMAND_PROCESSOR_BUFFER_SIZE 10    // how many messages can be stored in the buffer for processing. 10 is already overkill
#define DEBUG_COMMAND_PROCESSOR

TaskHandle_t Handle_task_cmd_processor = NULL;
QueueHandle_t queue_uart;
QueueHandle_t queue_bluetooth;

typedef enum command_source_t
{
	NONE,
	SERIAL0,
	SERIAL1,
	BLUETOOTH,
} command_source;

// #endregion

// #region Coisas da GPIO

static const char *TAG_GPIO = "GPIO STUFF";
#define BLINK_GPIO GPIO_NUM_23
static uint8_t s_led_state = 0;
int system_status = -1;    //-1 = error, 0 = ok but not connected, 1 = connected

void setup_gpio()
{
	ESP_LOGI(TAG, "running setup_gpio()");
	gpio_reset_pin(BLINK_GPIO);
	/* Set the GPIO as a push/pull output */
	gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(BLINK_GPIO, 0);
}

TaskHandle_t Task_led_handle;    // Esse handle não funciona se estiver em outro contexto, utilizei a variavel booleana "Serial2InUse"
// portMUX_TYPE myMutex2 = portMUX_INITIALIZER_UNLOCKED;
void Task_LED(void *pvParameters)
{
	ESP_LOGI(ITALO_TAG, "Task_LED() started");

	bool led_aceso = 0;
	bool led_apagado = 1;
	for (;;)
	{
		switch (system_status)
		{
			case -1:    // com problema, deve piscar 3x rapido e ficar apagado
				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(2000 / portTICK_PERIOD_MS);
				break;
			case 0:    // Não conectado, deve piscar rapido
				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				break;
			case 1:    // Conectado, deve piscar lento
				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(200 / portTICK_PERIOD_MS);

				gpio_set_level(BLINK_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(200 / portTICK_PERIOD_MS);

				gpio_set_level(BLINK_GPIO, led_apagado);
				vTaskDelay(1400 / portTICK_PERIOD_MS);
				break;
		}
	}

	//   Serial.print("Task3 running on core ");
	//   Serial.println(xPortGetCoreID());

	//   for (;;)
	//   {
	//      if ((SerialBT.hasClient() > 0))
	//     {
	//         digitalWrite(23,LOW);
	//         vTaskDelay(100 / portTICK_PERIOD_MS);
	//         digitalWrite(23,HIGH);
	//         vTaskDelay(200 / portTICK_PERIOD_MS);
	//         digitalWrite(23,LOW);
	//         vTaskDelay(100 / portTICK_PERIOD_MS);
	//         digitalWrite(23,HIGH);
	//         vTaskDelay(1600 / portTICK_PERIOD_MS);
	//     }
	//     else //nao conectado
	//     {
	//         digitalWrite(23,LOW);
	//         vTaskDelay(100 / portTICK_PERIOD_MS);
	//         digitalWrite(23,HIGH);
	//         vTaskDelay(100 / portTICK_PERIOD_MS);
	//     }
	// vTaskDelay(30000 / portTICK_PERIOD_MS); // consulta mais rapido para nao perder cortes
	// }
}

// #endregion

// #region COISAS DO BLUETOOTH

static uint32_t _spp_client_handle = 0;    // Criei esse handle para monitorar qual a conexao atual, e usar o SPP_WRITE fora do callback

static const esp_spp_mode_t esp_spp_mode = ESP_SPP_MODE_CB;

static struct timeval time_new, time_old;
static long data_num = 0;

static const esp_spp_sec_t sec_mask = ESP_SPP_SEC_AUTHENTICATE;
static const esp_spp_role_t role_slave = ESP_SPP_ROLE_SLAVE;

static char *bda2str(uint8_t *bda, char *str, size_t size)
{
	if (bda == NULL || str == NULL || size < 18)
	{
		return NULL;
	}

	uint8_t *p = bda;
	sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", p[0], p[1], p[2], p[3], p[4], p[5]);
	return str;
}

int getchipID()
{
	// Essa porra finalmente funfou aleluia caraio. q trem mal documentado da porra
	const uint8_t *point = esp_bt_dev_get_address();
	char str[3];
	for (int i = 0; i < 6; i++)
	{
		sprintf(str, "%02X", (int)point[i]);
		printf(str);
		if (i < 5)
		{
			printf(":");
		}
	}
	return 1;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
	char bda_str[18] = {0};

	switch (event)
	{
		case ESP_SPP_INIT_EVT:
			if (param->init.status == ESP_SPP_SUCCESS)
			{
				ESP_LOGI(SPP_TAG, "ESP_SPP_INIT_EVT");
				esp_spp_start_srv(sec_mask, role_slave, 0, SPP_SERVER_NAME);
			}
			else
			{
				ESP_LOGE(SPP_TAG, "ESP_SPP_INIT_EVT status:%d", param->init.status);
			}
			break;
		case ESP_SPP_DISCOVERY_COMP_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
			break;
		case ESP_SPP_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
			system_status = 1;
			break;
		case ESP_SPP_CLOSE_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT status:%d handle:%d close_by_remote:%d", param->close.status, param->close.handle, param->close.async);
			system_status = 0;
			break;
		case ESP_SPP_START_EVT:
			if (param->start.status == ESP_SPP_SUCCESS)
			{
				ESP_LOGI(SPP_TAG, "ESP_SPP_START_EVT handle:%d sec_id:%d scn:%d", param->start.handle, param->start.sec_id, param->start.scn);

				// O trecho a seguir é uma gambiarra para pegar o mac address,  muito mal documentado também.

				const uint8_t *point = esp_bt_dev_get_address();
				char full_mac_address[20];
				//            E0:5A:1B:41:7C:AA
				sprintf(full_mac_address, "%02X:%02X:%02X:%02X:%02X:%02X", (int)point[0], (int)point[1], (int)point[2], (int)point[3], (int)point[4], (int)point[5]);
				// printf("full_mac_address BRABA = %s\n", full_mac_address);

				char btname2[31];
				snprintf(btname2, sizeof(btname2), "xBTSAT_%02X%02X%02X", (int)point[3], (int)point[4], (int)point[5]);
				// printf("ideia  = %s\n", btname2);
				ESP_LOGI(SPP_TAG, "Bluetooth name is: %s , MAC ADDRESS: %s", btname2, full_mac_address);
				// for (int i = 0; i < 6; i++)
				// {
				//     sprintf(str, "%02X", (int)point[i]);
				//     printf(str);
				//      if (i < 5)
				//      {
				//      printf(":");
				//      }
				// }

				// char btname[31];
				// snprintf(btname, sizeof(btname), "xBTSAT_%d", getchipID());

				esp_bt_dev_set_device_name(btname2);
				esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
			}
			else
			{
				ESP_LOGE(SPP_TAG, "ESP_SPP_START_EVT status:%d", param->start.status);
			}
			break;
		case ESP_SPP_CL_INIT_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CL_INIT_EVT");
			break;
		case ESP_SPP_DATA_IND_EVT:
			/*
			 * We only show the data in which the data length is less than 128 here. If you want to print the data and
			 * the data rate is high, it is strongly recommended to process them in other lower priority application task
			 * rather than in this callback directly. Since the printing takes too much time, it may stuck the Bluetooth
			 * stack and also have a effect on the throughput!
			 */
			ESP_LOGI(SPP_TAG, "ESP_SPP_DATA_IND_EVT len:%d handle:%d", param->data_ind.len, param->data_ind.handle);

			// Prog do italo
			char buf[256];
			char spp_data[256];
			if (param->data_ind.len < 256)
			{
				snprintf(buf, (size_t)param->data_ind.len, (char *)param->data_ind.data);
				// uart_write_bytes(UART_NUM, buf, strlen(buf));
				// ESP_LOGI(ITALO_TAG, "SPP RX: %s\r\n", buf);

				ESP_LOGI("", "Sending to queue_bluetooth: %s", buf);
				xQueueSend(queue_bluetooth, (void *)buf, (TickType_t)10);

				// printf("RX DATA: %s\n", buf);
				// uart_write_bytes(UART_NUM, (const char *)buf, (param->data_ind.len) -1);
				// sprintf(spp_data, "ACK: %d chars\n", param->data_ind.len);
				// esp_spp_write(param->write.handle, strlen(spp_data), (uint8_t *)spp_data);
			}
			break;
		case ESP_SPP_CONG_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
			break;
		case ESP_SPP_WRITE_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
			break;
		case ESP_SPP_SRV_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_OPEN_EVT status:%d handle:%d, rem_bda:[%s]", param->srv_open.status, param->srv_open.handle,
			         bda2str(param->srv_open.rem_bda, bda_str, sizeof(bda_str)));

			ESP_LOGI(ITALO_TAG, "New client handle = %d", _spp_client_handle);
			_spp_client_handle = param->srv_open.handle;
			system_status = 1;

			gettimeofday(&time_old, NULL);
			break;
		case ESP_SPP_SRV_STOP_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_SRV_STOP_EVT");
			_spp_client_handle = 0;
			ESP_LOGI(ITALO_TAG, "Client disconnected, new client handle = %d", _spp_client_handle);
			system_status = 0;
			break;
		case ESP_SPP_UNINIT_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_UNINIT_EVT");
			break;
		default:
			break;
	}
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
	char bda_str[18] = {0};

	switch (event)
	{
		case ESP_BT_GAP_AUTH_CMPL_EVT:
		{
			if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
			{
				ESP_LOGI(SPP_TAG, "authentication success: %s bda:[%s]", param->auth_cmpl.device_name, bda2str(param->auth_cmpl.bda, bda_str, sizeof(bda_str)));
			}
			else
			{
				ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
			}
			break;
		}
		case ESP_BT_GAP_PIN_REQ_EVT:
		{
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_PIN_REQ_EVT min_16_digit:%d", param->pin_req.min_16_digit);
			if (param->pin_req.min_16_digit)
			{
				ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
				esp_bt_pin_code_t pin_code = {0};
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
			}
			else
			{
				ESP_LOGI(SPP_TAG, "Input pin code: 1234");
				esp_bt_pin_code_t pin_code;
				pin_code[0] = '1';
				pin_code[1] = '2';
				pin_code[2] = '3';
				pin_code[3] = '4';
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
			}
			break;
		}

#if (CONFIG_BT_SSP_ENABLED == true)
		case ESP_BT_GAP_CFM_REQ_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
			esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
			break;
		case ESP_BT_GAP_KEY_NOTIF_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
			break;
		case ESP_BT_GAP_KEY_REQ_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
			break;
#endif

		case ESP_BT_GAP_MODE_CHG_EVT:
			ESP_LOGI(SPP_TAG, "ESP_BT_GAP_MODE_CHG_EVT mode:%d bda:[%s]", param->mode_chg.mode, bda2str(param->mode_chg.bda, bda_str, sizeof(bda_str)));
			break;

		default:
		{
			ESP_LOGI(SPP_TAG, "event: %d", event);
			break;
		}
	}
	return;
}

// #endregion

// #region COISAS DO SERIAL

/**
 * @brief This task will get UART data and send it to Blueooth
 *
 * @param arg
 */
static void echo_task(void *arg)
{
	uint8_t *data = (uint8_t *)malloc(BUF_SIZE);    // Configure a temporary buffer for the incoming data
	while (1)
	{
		int len = uart_read_bytes(UART_NUM, data, (BUF_SIZE - 1), 20 / portTICK_PERIOD_MS);
		// uart_write_bytes(UART_NUM, (const char *)data, len); // Write data back to the UART
		if (len)
		{
			esp_spp_cb_param_t *param;
			data[len] = '\0';
			ESP_LOGI(TAG, "RECV From RX2 and sent to queue_uart: %s", (char *)data);

			// esp_spp_write(param->write.handle, sizeof(data), data);
			// acho q nao funfa pq isso tem que ta dentro do callback. talvez um callback pra qdo quiser escrever.
			// https://higaski.at/aoihashi-esp32-bluetooth-serial-module/

			xQueueSend(queue_uart, (void *)data, (TickType_t)10);

			// THIS THREE LINES WERE ENABLED, BUT NOW THEY WILL BE SENT BY THE COMMAND PROCESSOR.
			// uint32_t UART_len = len;
			// esp_err_t err = esp_spp_write(_spp_client_handle, UART_len, data);
			// ESP_LOGI(TAG, "Written to SPP[%d]: %s", _spp_client_handle, (char *)data);
		}

		// é aqui que tenho que criar o meu programa
		// 1 ler o que ta no bt
		// 2 printar no serial,  Seria melhor se fosse char por char, eu acho.
	}
}

// #endregion

void task_cmd_processor(void *arg)
{
	uint8_t rxbuff[256];
	bool command_waiting = false;
	bool was_an_command = false;
	command_source source;
	while (1)
	{
		// ========================= GET THE BUFFER DATA =================================
		if (xQueueReceive(queue_uart, &(rxbuff), (TickType_t)5))
		{
			source = SERIAL0;
			command_waiting = true;
#ifdef DEBUG_COMMAND_PROCESSOR
			ESP_LOGI("", "got a data from UART queue!  ===  %s", rxbuff);
#endif
		}
		else if (xQueueReceive(queue_bluetooth, &(rxbuff), (TickType_t)5))
		{
			source = BLUETOOTH;
			command_waiting = true;
#ifdef DEBUG_COMMAND_PROCESSOR
			ESP_LOGI("", "got a data from BT queue!  ===  %s", rxbuff);
#endif
		}
		// ========================= TRY TO PROCESS COMMANDS =================================
		if (command_waiting == true)
	 {
#ifdef ENABLE_AT_PROTOCOL_SUPPORT
			was_an_command = process_at_protocol(rxbuff, source);
#endif
			if (was_an_command)
			{
				ESP_LOGI("", "Message was an AT and wont be fowarded");
				// discart the message and does not foward it to the target.
			}
			else
			{
				uint32_t UART_len = sizeof(rxbuff);
				// uint8_t data[rxbuff];
				// snprintf(data, sizeof(data), "%s",rxbuff);
				int len1 = strlen((char*)rxbuff);
				esp_err_t err = esp_spp_write(_spp_client_handle, len1, (uint8_t *)rxbuff);
				ESP_LOGI(TAG, "Written to SPP[%d]: %s", _spp_client_handle, (char *)rxbuff);

				// reply_to_source("AT+ERROR - Command unknown\r\n", source);
				//  foward the message
			}
			command_waiting = false;
		}
	}
	vTaskDelay(pdMS_TO_TICKS(1));
}

void setup_command_processor()
{
	uint8_t txbuff[256];
	queue_uart = xQueueCreate(COMMAND_PROCESSOR_BUFFER_SIZE, sizeof(txbuff));
	queue_bluetooth = xQueueCreate(COMMAND_PROCESSOR_BUFFER_SIZE, sizeof(txbuff));
	if (queue_uart == NULL)
	{
		ESP_LOGE("", "Command_processor initialized - Unable to create queue_uart");
		return;
	}
	if (queue_bluetooth == NULL)
	{
		ESP_LOGE("", "Command_processor initialized - Unable to create queue_bluetooth");
		return;
	}
	xTaskCreatePinnedToCore(task_cmd_processor, "task_cmd_processor", 4096, NULL, 10, &Handle_task_cmd_processor, 1);
	ESP_LOGI("", "Command_processor initialized");
}

extern "C" void app_main(void)
{
	setup_gpio();

	/* #region  Criando o bluetooth*/
	char bda_str[18] = {0};
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_init()) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bluedroid_enable()) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}

	if ((ret = esp_spp_init(esp_spp_mode)) != ESP_OK)
	{
		system_status = -1;
		ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
		return;
	}
	else
	{
		system_status = 0;
	}

#if (CONFIG_BT_SSP_ENABLED == true)
	/* Set default parameters for Secure Simple Pairing */
	esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
	esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
	esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));
#endif

	/*
	 * Set default parameters for Legacy Pairing
	 * Use variable pin, input pin code when pairing
	 */
	esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
	esp_bt_pin_code_t pin_code;
	esp_bt_gap_set_pin(pin_type, 0, pin_code);

	ESP_LOGI(SPP_TAG, "Own address:[%s]", bda2str((uint8_t *)esp_bt_dev_get_address(), bda_str, sizeof(bda_str)));

	/* #endregion */

	/* #region  Criando o Serial*/
	/* Configure parameters of an UART driver,
	 * communication pins and install the driver */
	uart_config_t uart_config = {
	    .baud_rate = ECHO_UART_BAUD_RATE,
	    .data_bits = UART_DATA_8_BITS,
	    .parity = UART_PARITY_DISABLE,
	    .stop_bits = UART_STOP_BITS_1,
	    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
	    .source_clk = UART_SCLK_DEFAULT,
	};
	int intr_alloc_flags = 0;

#if CONFIG_UART_ISR_IN_IRAM
	intr_alloc_flags = ESP_INTR_FLAG_IRAM;
#endif

	ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, intr_alloc_flags));
	ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(UART_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, ECHO_TEST_RTS, ECHO_TEST_CTS));

	xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, 10, NULL);

	/* #endregion */

	/* #region Criando task do LED */
	// xTaskCreate(led_task, "led_task", ECHO_TASK_STACK_SIZE, NULL, 1, NULL);
	xTaskCreatePinnedToCore(Task_LED,         /* Task function. */
	                        "Task_LED",       /* name of task. */
	                        2048,             /* Stack size of task */
	                        NULL,             /* parameter of the task */
	                        1,                /* priority of the task */
	                        &Task_led_handle, /* Task handle to keep track of created task */
	                        1);               /* pin task to core 0 */

	/* #endregion */

	setup_command_processor();

	// Fim do "setup", a partir daqui tudo roda em task!?
}