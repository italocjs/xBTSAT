/**
 * @file ota.h
 * @author Italo Soares (italocjs@live.com)
 * @brief Code needed for the WIFI OTA capability.
 * @version 1.4
 * @date 2023-05-15
 * 2023-05-16 12:20:51 code cleanup and released as V1.4
 * @copyright Copyright (c) 2023
 */
#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>

#define OTA_TAG "OTA_TAG"
bool ota_started = false;

extern const char *host;
extern const char *ssid;    // = "xBTSAT_OTA";
extern const char *password;

WebServer server(80);
#include "ota_webpage.h"    //Load the webpage resource

void onJavaScript(void)
{
	Serial.println("onJavaScript(void)");
	server.setContentLength(jquery_min_js_v3_2_1_gz_len);
	server.sendHeader(F("Content-Encoding"), F("gzip"));
	server.send_P(200, "text/javascript", jquery_min_js_v3_2_1_gz, jquery_min_js_v3_2_1_gz_len);
}

int getchipID()
{
	uint32_t chipId = 0;
	for (int i = 0; i < 17; i = i + 8)
	{
		chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
	}
	return chipId;
}

void start_ota()
{
	char ssid_2[15];
	snprintf(ssid_2, sizeof(ssid_2), "xBTSAT_%d", getchipID());
	ssid = ssid_2;

	IPAddress local_ip(192, 168, 1, 1);
	IPAddress local_mask(255, 255, 255, 0);
	IPAddress gateway(192, 168, 1, 1);
	WiFi.softAP(ssid, password);
	WiFi.softAPConfig(local_ip, gateway, local_mask);

	/*use mdns for host name resolution  http://esp32.local*/
	if (!MDNS.begin(host))
	{
		ESP_LOGE(OTA_TAG, "Error setting up MDNS responder!");
		while (1)
		{
			delay(1000);
		}
	}
	ESP_LOGI(OTA_TAG, "mDNS responder started");

	/*return javascript jquery */
	server.on("/jquery.min.js", HTTP_GET, onJavaScript);

	/*return index page which is stored in serverIndex */
	server.on("/", HTTP_GET,
	          []()
	          {
		          server.sendHeader("Connection", "close");
#ifdef REQUIRE_LOGIN
		          server.send(200, "text/html", loginIndex);
#else
		          server.send(200, "text/html", serverIndex);    //Go diretly to upload page
#endif
	          });
	server.on("/serverIndex", HTTP_GET,
	          []()
	          {
		          server.sendHeader("Connection", "close");
		          server.send(200, "text/html", serverIndex);
	          });
	/*handling uploading firmware file */
	server.on(
	    "/update", HTTP_POST,
	    []()
	    {
		    server.sendHeader("Connection", "close");
		    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
		    ESP.restart();
	    },
	    []()
	    {
		    HTTPUpload &upload = server.upload();
		    if (upload.status == UPLOAD_FILE_START)
		    {
			    Serial.printf("Update: %s\n", upload.filename.c_str());
			    if (!Update.begin(UPDATE_SIZE_UNKNOWN))
			    {    // start with max available size
				    Update.printError(Serial);
			    }
		    }
		    else if (upload.status == UPLOAD_FILE_WRITE)
		    {
			    /* flashing firmware to ESP*/
			    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize)
			    {
				    Update.printError(Serial);
			    }
		    }
		    else if (upload.status == UPLOAD_FILE_END)
		    {
			    if (Update.end(true))
			    {    // true to set the size to the current progress
				    ESP_LOGI(OTA_TAG, "Update Success: %u\nRebooting...\n", upload.totalSize);
			    }
			    else
			    {
				    Update.printError(Serial);
				    ESP_LOGE(OTA_TAG, "Atualizacao falhou");
			    }
		    }
	    });
	server.begin();
}

TaskHandle_t Task_ota_handle;
void Task_ota(void *pvParameters)
{
	ESP_LOGI(OTA_TAG, "started");
	start_ota();
	ota_started = true;
	for (;;)
	{
		server.handleClient();
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
}

void setup_ota()
{
	if (ota_started == true)
	{
		ESP_LOGI("", "OTA Already started");
		return;
	}
	xTaskCreatePinnedToCore(Task_ota,         /* Task function. */
	                        "Task_ota",       /* name of task. */
	                        16384,            /* Stack size of task */
	                        NULL,             /* parameter of the task */
	                        1,                /* priority of the task */
	                        &Task_ota_handle, /* Task handle to keep track of created task */
	                        1);               /* pin task to core 0 */

	Serial.println("OTA STARTED, IP 192.168.1.1");
}