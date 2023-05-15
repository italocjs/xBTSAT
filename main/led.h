/**
 * @file led.h
 * @author Italo Soares (italocjs@live.com)
 * @brief
 * @version 0.1
 * @date 2023-05-15
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once
#include <Arduino.h>
#include <config.h>

TaskHandle_t Task_led_handle;
void Task_LED(void *pvParameters)
{
	ESP_LOGI("", "Task_LED() started");

	bool led_aceso = 0;
	bool led_apagado = 1;
	for (;;)
	{
		switch (system_status)
		{
			case -1:    // com problema, deve piscar 3x rapido e ficar apagado
				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(250 / portTICK_PERIOD_MS);

				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(2000 / portTICK_PERIOD_MS);
				break;
			case 0:    // Não conectado, deve piscar rapido
				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				break;
			case 1:    // Conectado, deve piscar lento
				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(200 / portTICK_PERIOD_MS);

				gpio_set_level(LED_GPIO, led_aceso);
				vTaskDelay(100 / portTICK_PERIOD_MS);
				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(200 / portTICK_PERIOD_MS);

				gpio_set_level(LED_GPIO, led_apagado);
				vTaskDelay(1400 / portTICK_PERIOD_MS);
				break;
		}
	}
}

void setup_led_task()
{
	gpio_reset_pin(LED_GPIO);
	gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(LED_GPIO, 0);

	xTaskCreatePinnedToCore(Task_LED,         /* Task function. */
	                        "Task_LED",       /* name of task. */
	                        2048,             /* Stack size of task */
	                        NULL,             /* parameter of the task */
	                        1,                /* priority of the task */
	                        &Task_led_handle, /* Task handle to keep track of created task */
	                        1);               /* pin task to core 0 */
}
