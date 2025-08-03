#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "http_server.h"

#define HTML_PATH "/spiffs/index.html"
#define MAX_WS_CLIENTS 4

static const char *TAG = "http_server";
static httpd_handle_t server = NULL;
static int client_count = 0;
static int client_fds[MAX_WS_CLIENTS] = {0};
static SemaphoreHandle_t client_lock = NULL;

/**
 * @brief Add a new client to the list
 */
static void add_client(int fd) {
    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (client_fds[i] == 0) {
            client_fds[i] = fd;
            client_count++;
            break;
        }
    }
    xSemaphoreGive(client_lock);
    ESP_LOGI(TAG, "New client connected (fd=%d), total clients: %d", fd, client_count);
}

/**
 * @brief Remove a client from the list
 */
static void remove_client(int fd) {
    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (client_fds[i] == fd) {
            client_fds[i] = 0;
            client_count--;
            break;
        }
    }
    xSemaphoreGive(client_lock);
    ESP_LOGI(TAG, "Client disconnected (fd=%d), total clients: %d", fd, client_count);
}

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

/**
 * @brief WebSocket events handler using ESP-IDF's built-in WebSocket support
 */
static esp_err_t ws_handler(httpd_req_t *req) {
    int sock = httpd_req_to_sockfd(req);
    
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake received for WebSocket");
        
        // Add the client to our list
        add_client(sock);
        return ESP_OK;
    }
    
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // Get the WebSocket frame header
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WebSocket frame receive failed: %d", ret);
        return ret;
    }
    
    // Check for close frame
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Received WebSocket close frame");
        remove_client(sock);
        return ESP_OK;
    }
    
    // If the frame has data, allocate memory for it
    if (ws_pkt.len) {
        ws_pkt.payload = malloc(ws_pkt.len + 1);
        if (!ws_pkt.payload) {
            ESP_LOGE(TAG, "Failed to allocate memory for WebSocket payload");
            return ESP_ERR_NO_MEM;
        }
        
        // Receive the WebSocket frame data
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            free(ws_pkt.payload);
            ESP_LOGE(TAG, "WebSocket frame receive failed: %d", ret);
            return ret;
        }
        
        // Null-terminate the received data (if it's text)
        if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
            ((char*)ws_pkt.payload)[ws_pkt.len] = 0;
            ESP_LOGI(TAG, "WebSocket received: %s", (char*)ws_pkt.payload);
        }
        
        // Echo the message back (just for testing)
        ret = httpd_ws_send_frame(req, &ws_pkt);
        free(ws_pkt.payload);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "WebSocket send failed: %d", ret);
            return ret;
        }
    }
    
    return ESP_OK;
}

static httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.lru_purge_enable = true; // Enable purging of old connections
    
    // Initialize client lock
    if (!client_lock) {
        client_lock = xSemaphoreCreateMutex();
        if (!client_lock) {
            ESP_LOGE(TAG, "Failed to create client lock");
            return NULL;
        }
    }
    
    ESP_LOGI(TAG, "Starting server on port: %d", config.server_port);
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register URI handlers
        httpd_uri_t index_uri = {
            .uri      = "/",
            .method   = HTTP_GET,
            .handler  = index_get_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(server, &index_uri);
        
        // Register WebSocket handler
        httpd_uri_t ws_uri = {
            .uri      = "/ws",
            .method   = HTTP_GET,
            .handler  = ws_handler,
            .user_ctx = NULL,
            .is_websocket = true
        };
        httpd_register_uri_handler(server, &ws_uri);
        
        ESP_LOGI(TAG, "HTTP server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return NULL;
    }
    
    return server;
}

static void http_server_task(void *pvParameters) {
    ESP_LOGI(TAG, "Starting HTTP server task on core %d", xPortGetCoreID());
    start_webserver();
    vTaskDelete(NULL); // Task can delete itself after starting server
}

void start_http_server(void) {
    xTaskCreatePinnedToCore(http_server_task, "http_server", 8192, NULL, 5, NULL, 1); // Pin to core 1, increased stack size
}

void update_websocket_data(int counter) {
    if (!server || client_count == 0) {
        return; // No server or no clients
    }
    
    // Prepare the JSON data
    char json_data[64];
    snprintf(json_data, sizeof(json_data), "{\"counter\":%d}", counter);
    
    // Create a WebSocket frame
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.payload = (uint8_t*)json_data;
    ws_pkt.len = strlen(json_data);
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // Send to all connected clients
    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = client_fds[i];
        if (fd) {
            esp_err_t ret = httpd_ws_send_frame_async(server, fd, &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGW(TAG, "Failed to send data to client %d: %d", fd, ret);
                // If the client is unreachable, mark it as disconnected
                client_fds[i] = 0;
                client_count--;
                ESP_LOGW(TAG, "Client %d removed, total clients: %d", fd, client_count);
            } else {
                ESP_LOGI(TAG, "Sent counter %d to client %d", counter, fd);
            }
        }
    }
    xSemaphoreGive(client_lock);
}
