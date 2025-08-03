#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "http_server.h"

#define HTML_PATH "/spiffs/index.html"

static const char *TAG = "http_server";


static esp_err_t index_get_handler(httpd_req_t *req) {
    FILE *file = fopen(HTML_PATH, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open %s", HTML_PATH);
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }
    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "text/html");
    while ((read_bytes = fread(buffer, 1, sizeof(buffer), file)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }
    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
    }
    return server;
}

static void http_server_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting HTTP server task on core %d", xPortGetCoreID());
    start_webserver();
    vTaskDelete(NULL); // Task can delete itself after starting server
}

void start_http_server(void) {
    xTaskCreatePinnedToCore(http_server_task, "http_server", 4096, NULL, 5, NULL, 1); // Pin to core 1
}
