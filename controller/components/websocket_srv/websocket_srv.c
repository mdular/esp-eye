  // Diagnostic logging for websocket connection stability
 #include "esp_log.h"
 static const char *DIAG_TAG = "websocket_diag";
 #include "wifi_connect.h"
 #include <stdarg.h>
 #include <string.h>
 #include <stdio.h>
 #include <errno.h>
 #include <inttypes.h>
 #include <stdlib.h>
 #include <math.h>   // for isfinite used in command validation
 #include "esp_http_server.h"
 #include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_spiffs.h"
#include "websocket_srv.h"
#include "common_types.h"
#include "controller.h"

#define HTML_PATH "/spiffs/index.html"
#define MAX_WS_CLIENTS 4
#define WEB_TASK_STACK_SIZE 6144
#define WEB_TASK_PRIORITY 3
#define TELEMETRY_INTERVAL_MS 200  // 5Hz telemetry rate
#define CLIENT_PRUNE_INTERVAL_MS 5000  // Check for stale clients every 5 seconds
#define WS_LOG_MIN_LEVEL LOG_LEVEL_WARNING

static const char *TAG = "websocket_srv";
static httpd_handle_t server = NULL;
static int client_count = 0;
static int client_fds[MAX_WS_CLIENTS] = {0};
static SemaphoreHandle_t client_lock = NULL;
static TaskHandle_t web_task_handle = NULL;
static QueueHandle_t telemetry_queue = NULL;
#if ENABLE_WS_LOGS
static QueueHandle_t log_queue = NULL;
// Forward declaration for custom log vprintf
static int websocket_log_vprintf(const char *fmt, va_list args);
#endif

/* Utility extractors for lightweight JSON-ish command parsing (no cJSON) */
static int ws_extract_int(const char *buf, const char *key, int *out) {
    const char *p = strstr(buf, key);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p==' '||*p=='\t') p++;
    char *endptr = NULL;
    long v = strtol(p, &endptr, 10);
    if (endptr == p) return 0;
    *out = (int)v;
    return 1;
}
static int ws_extract_float(const char *buf, const char *key, float *out) {
    const char *p = strstr(buf, key);
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p==' '||*p=='\t') p++;
    char *endptr = NULL;
    double v = strtod(p, &endptr);
    if (endptr == p) return 0;
    *out = (float)v;
    return 1;
}
static int ws_extract_cmd_token(const char *buf, char *out, size_t out_sz) {
    const char *p = strstr(buf, "\"cmd\"");
    if (!p) return 0;
    p = strchr(p, ':');
    if (!p) return 0;
    p++;
    while (*p==' '||*p=='\t') p++;
    if (*p != '\"') return 0;
    p++;
    size_t i=0;
    while (*p && *p!='\"' && i < out_sz-1) {
        out[i++] = *p++;
    }
    out[i] = 0;
    return i>0;
}

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
ESP_LOGI(DIAG_TAG, "Heap free: %u, WiFi connected: %d", esp_get_free_heap_size(), wifi_is_connected());
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
    ESP_LOGI(DIAG_TAG, "Heap free: %u, WiFi connected: %d", esp_get_free_heap_size(), wifi_is_connected());
}

/**
 * @brief Check if a client is still connected and remove if not
 */
static void check_and_remove_stale_clients(void) {
    xSemaphoreTake(client_lock, portMAX_DELAY);
    
    int removed = 0;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (client_fds[i] != 0) {
            // Create a dummy packet to check if client is still connected
            char dummy = 0;
            int ret = httpd_ws_send_frame_async(server, client_fds[i], &(httpd_ws_frame_t){
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_PING,
                .payload = (uint8_t*)&dummy,
                .len = 0
            });
            
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Client %d (fd=%d) ping failed, removing", i, client_fds[i]);
                client_fds[i] = 0;
                client_count--;
                removed++;
            }
        }
    }
    
    xSemaphoreGive(client_lock);
    
    if (removed > 0) {
        ESP_LOGI(TAG, "Removed %d stale client(s), total clients: %d", removed, client_count);
    }
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
    
    // Diagnostic: log before frame header receive
    ESP_LOGD(DIAG_TAG, "About to call httpd_ws_recv_frame (header), sock=%d", sock);
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    
    // Diagnostic: log after frame header receive
    ESP_LOGD(DIAG_TAG, "httpd_ws_recv_frame (header) returned %d, type=%d, len=%d", ret, ws_pkt.type, ws_pkt.len);
    if (ret != ESP_OK) {
        // Handle any error gracefully - could be masking issues or connection issues
        if (ret == ESP_ERR_INVALID_STATE || ret == ESP_FAIL) {
            // This could be a frame masking issue or just a connection reset
            ESP_LOGW(TAG, "WebSocket frame issue (err: %d, type: %d), client might be disconnecting", 
                    ret, ws_pkt.type);
            
            // Check if client is still connected before removing
            char dummy = 0;
            esp_err_t ping_ret = httpd_ws_send_frame_async(server, sock, &(httpd_ws_frame_t){
                .final = true,
                .fragmented = false,
                .type = HTTPD_WS_TYPE_PING,
                .payload = (uint8_t*)&dummy,
                .len = 0
            });
            
            if (ping_ret != ESP_OK) {
                ESP_LOGI(TAG, "Client ping failed after frame error, removing client");
                remove_client(sock);
            }
            return ESP_OK;  // Don't propagate the error
        }
        
        ESP_LOGD(TAG, "WebSocket frame receive failed: %d, type: %d, len: %d", 
                ret, ws_pkt.type, ws_pkt.len);
        
        // Remove client for any other errors indicating disconnection
        remove_client(sock);
        return ESP_OK;  // Return OK to prevent the httpd server from closing the socket
    }
    
    // Check frame type
    if (ws_pkt.type == HTTPD_WS_TYPE_CLOSE) {
        ESP_LOGI(TAG, "Received WebSocket close frame");
        remove_client(sock);
        return ESP_OK;
    } else if (ws_pkt.type == HTTPD_WS_TYPE_PING) {
        // Respond to ping with pong
        httpd_ws_frame_t pong_frame;
        memset(&pong_frame, 0, sizeof(httpd_ws_frame_t));
        pong_frame.type = HTTPD_WS_TYPE_PONG;
        pong_frame.payload = NULL;
        pong_frame.len = 0;
        httpd_ws_send_frame(req, &pong_frame);
        ESP_LOGD(TAG, "Ping received, sent pong");
        return ESP_OK;
    } else if (ws_pkt.type == HTTPD_WS_TYPE_PONG) {
        ESP_LOGD(TAG, "Pong received from client");
        return ESP_OK;
    } else if (ws_pkt.type == HTTPD_WS_TYPE_BINARY) {
        ESP_LOGW(TAG, "Binary WebSocket frame received (%d bytes), not supported", (int)ws_pkt.len);
        return ESP_OK;  // Acknowledge but don't process binary frames
    } else if (ws_pkt.type != HTTPD_WS_TYPE_TEXT) {
        ESP_LOGW(TAG, "Unknown WebSocket frame type: %d", ws_pkt.type);
        return ESP_OK;
    }
    
    // Diagnostic: log before payload allocation
    ESP_LOGD(DIAG_TAG, "Allocating payload buffer of size %d", ws_pkt.len + 1);

    // Allocate memory for the payload based on the frame length
    uint8_t *payload = malloc(ws_pkt.len + 1);
    if (payload == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for WebSocket payload");
        ESP_LOGD(DIAG_TAG, "Payload allocation failed for len=%d", ws_pkt.len);
        return ESP_ERR_NO_MEM;
    }

    // Diagnostic: log before payload receive
    ESP_LOGD(DIAG_TAG, "About to call httpd_ws_recv_frame (payload), sock=%d, len=%d", sock, ws_pkt.len);

    // Receive the payload
    ws_pkt.payload = payload;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);

    // Diagnostic: log after payload receive
    ESP_LOGD(DIAG_TAG, "httpd_ws_recv_frame (payload) returned %d, type=%d, len=%d", ret, ws_pkt.type, ws_pkt.len);

    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WebSocket payload receive failed: %d, type: %d, len: %d",
                ret, ws_pkt.type, ws_pkt.len);
        ESP_LOGD(DIAG_TAG, "Payload receive error: ret=%d, type=%d, len=%d", ret, ws_pkt.type, ws_pkt.len);
        free(payload);

        // Diagnostic: log before ping on payload error
        ESP_LOGD(DIAG_TAG, "Sending ping to client after payload error, sock=%d", sock);

        // Check if client is still connected before removing
        char dummy = 0;
        esp_err_t ping_ret = httpd_ws_send_frame_async(server, sock, &(httpd_ws_frame_t){
            .final = true,
            .fragmented = false,
            .type = HTTPD_WS_TYPE_PING,
            .payload = (uint8_t*)&dummy,
            .len = 0
        });

        // Diagnostic: log after ping attempt
        ESP_LOGD(DIAG_TAG, "Ping after payload error returned %d", ping_ret);

        if (ping_ret != ESP_OK) {
            ESP_LOGI(TAG, "Client ping failed after payload error, removing client");
            remove_client(sock);
        }
        return ESP_OK;  // Don't propagate the error
    }
    
    // Null-terminate the payload string
    payload[ws_pkt.len] = 0;
    
    // Check for potentially binary/corrupted data
    bool is_printable = true;
    for (size_t i = 0; i < ws_pkt.len && i < 20; i++) {  // Check first 20 bytes max
        if (payload[i] < 32 && payload[i] != '\n' && payload[i] != '\r' && payload[i] != '\t') {
            is_printable = false;
            break;
        }
    }
    
    if (is_printable) {
        ESP_LOGI(TAG, "Received WebSocket message: %s", payload);
    } else {
        ESP_LOGW(TAG, "Received binary/corrupted WebSocket data (%d bytes)", (int)ws_pkt.len);
    }
    
    // Extended versioned command schema (no cJSON). Supported examples:
    // {"v":1,"cmd":"set","yaw_sp":1.5708}
    // {"v":1,"cmd":"motors","motors":1}
    // {"v":1,"cmd":"estop"}
    // {"v":1,"cmd":"level"}              // sets yaw_sp=0
    // {"v":1,"cmd":"point","yaw_sp":-0.7}
    // {"v":1,"cmd":"calib","op":"hall_zero"}  // unsupported -> error
    // Fields:
     //   v (required)        : schema version
     //   cmd (required)      : set|motors|estop|level|point|calib
     //   motors (optional)   : 0/1
     //   estop (optional)    : 0/1
     //   yaw_sp (optional)   : float radians (validated)
     //   op (for calib)      : operation keyword (future)
#ifndef WS_CMD_SCHEMA_VERSION
#define WS_CMD_SCHEMA_VERSION 1
#endif
#ifndef YAW_SETPOINT_ABS_MAX
#define YAW_SETPOINT_ABS_MAX 6.283f
#endif

    const char *text = (const char*)payload;

    bool schema_mode = (strstr(text, "\"v\"") && strstr(text, "\"cmd\""));
    int version = -1;
    bool version_ok = false;
    bool estop = false;
    bool motors_present = false;
    bool motors_enable = false;
    bool yaw_sp_present = false;
    float yaw_sp_value = 0.0f;
    char cmd_token[16] = {0};
    char op_token[16]  = {0};

    if (schema_mode) {
        if (ws_extract_int(text, "\"v\"", &version)) {
            version_ok = (version == WS_CMD_SCHEMA_VERSION);
        }
        ws_extract_cmd_token(text, cmd_token, sizeof(cmd_token));
        int tmp;
        if (ws_extract_int(text, "\"motors\"", &tmp)) {
            motors_present = true;
            motors_enable = (tmp != 0);
        }
        if (ws_extract_int(text, "\"estop\"", &tmp)) {
            estop = (tmp != 0);
        }
        if (ws_extract_float(text, "\"yaw_sp\"", &yaw_sp_value)) {
            yaw_sp_present = true;
        }
        // Extract calib op (simple)
        const char *op_pos = strstr(text, "\"op\"");
        if (op_pos) {
            const char *c = strchr(op_pos, ':');
            if (c) {
                c++;
                while (*c==' '||*c=='\t') c++;
                if (*c=='\"') {
                    c++;
                    size_t i=0;
                    while (*c && *c!='\"' && i < sizeof(op_token)-1) op_token[i++] = *c++;
                    op_token[i] = 0;
                }
            }
        }
        if (strcmp(cmd_token, "estop") == 0) {
            estop = true;
        }
    } else {
        // Legacy fallback (unchanged)
        if (strstr(text, "\"estop\"")) {
            if (strstr(text, "\"cmd\"") && strstr(text, "\"estop\"")) {
                estop = true;
            } else if (strstr(text, "\"estop\":true") || strstr(text, "\"estop\":1")) {
                estop = true;
            }
        }
        if (strstr(text, "\"motors\"")) {
            const char *m = strstr(text, "\"motors\"");
            if (m) {
                const char *colon = strchr(m, ':');
                if (colon) {
                    while (*++colon == ' ');
                    if (*colon == '1' || *colon == 't' || *colon == 'T') {
                        motors_present = true;
                        motors_enable = true;
                    } else if (*colon == '0' || *colon == 'f' || *colon == 'F') {
                        motors_present = true;
                        motors_enable = false;
                    }
                }
            }
        }
    }

    char response[256];
    esp_err_t action_result = ESP_OK;
    const char *err_code = NULL;

    if (schema_mode && !version_ok) {
        err_code = "bad_version";
    }
    if (!err_code && schema_mode && cmd_token[0] == 0) {
        err_code = "missing_cmd";
    }
    if (!err_code && yaw_sp_present) {
        if (!isfinite(yaw_sp_value) ||
            yaw_sp_value < -YAW_SETPOINT_ABS_MAX ||
            yaw_sp_value >  YAW_SETPOINT_ABS_MAX) {
            err_code = "invalid_value";
        }
    }

    if (!err_code && schema_mode) {
        if (strcmp(cmd_token,"set")==0) {
            /* combined field application */
        } else if (strcmp(cmd_token,"motors")==0) {
            if (!motors_present) err_code = "missing_field";
        } else if (strcmp(cmd_token,"estop")==0) {
            /* handled below */
        } else if (strcmp(cmd_token,"level")==0) {
            yaw_sp_present = true;
            yaw_sp_value = 0.0f;
        } else if (strcmp(cmd_token,"point")==0) {
            if (!yaw_sp_present) err_code = "missing_field";
        } else if (strcmp(cmd_token,"calib")==0) {
            /* future calib operations not yet implemented */
            err_code = "unsupported_cmd";
        } else {
            err_code = "unknown_cmd";
        }
    }

    if (err_code) {
        snprintf(response, sizeof(response),
                 "{\"status\":\"error\",\"err\":\"%s\"}", err_code);
    } else if (estop) {
        action_result = controller_emergency_stop();
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"cmd\":\"estop\",\"result\":%d}",
                 action_result == ESP_OK ? 1 : 0);
        ESP_LOGW(TAG, "Web command: estop");
    } else if (motors_present && (strcmp(cmd_token,"motors")==0 || strcmp(cmd_token,"set")==0)) {
        controller_set_motors_enabled(motors_enable);
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"cmd\":\"motors\",\"enabled\":%d}",
                 motors_enable ? 1 : 0);
        ESP_LOGI(TAG, "Web command: motors %s", motors_enable ? "ENABLED" : "DISABLED");
    } else if (yaw_sp_present && (strcmp(cmd_token,"set")==0 ||
                                  strcmp(cmd_token,"point")==0 ||
                                  strcmp(cmd_token,"level")==0)) {
        controller_set_yaw_setpoint(yaw_sp_value);
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"cmd\":\"%s\",\"yaw_sp\":%.4f}",
                 cmd_token[0]?cmd_token:"set", yaw_sp_value);
        ESP_LOGI(TAG, "Web command: %s yaw_sp=%.4f",
                 cmd_token[0]?cmd_token:"set", yaw_sp_value);
    } else if (!schema_mode && (motors_present || yaw_sp_present)) {
        if (motors_present) controller_set_motors_enabled(motors_enable);
        if (yaw_sp_present) controller_set_yaw_setpoint(yaw_sp_value);
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"legacy\":1}");
    } else {
        snprintf(response, sizeof(response),
                 "{\"status\":\"ok\",\"message\":\"noop\"}");
    }

    httpd_ws_frame_t resp_ws_pkt;
    memset(&resp_ws_pkt, 0, sizeof(httpd_ws_frame_t));
    resp_ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    resp_ws_pkt.payload = (uint8_t*)response;
    resp_ws_pkt.len = strlen(response);

    ret = httpd_ws_send_frame(req, &resp_ws_pkt);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WebSocket send failed: %d", ret);
    }

    free(payload);
    return ESP_OK;
}

/**
 * @brief Send telemetry data to all connected WebSocket clients
 */
static void send_telemetry_to_clients(const telemetry_data_t *data) {
    if (client_count == 0) {
        return;  // No clients connected
    }

    // Compact array layout:
    // t = [q0,q1,q2,q3,roll,pitch,yaw,m1,m2,status,seq,missed,lastHallUs]
    char json[512];
    int written = snprintf(
        json, sizeof(json),
        "{\"t\":[%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%ld,%lu,%lu,%" PRIu64 "]}",
        data->q0, data->q1, data->q2, data->q3,
        data->roll, data->pitch, data->yaw,
        data->motor1_speed, data->motor2_speed,
        (long)data->status,
        (unsigned long)data->imu_sequence,
        (unsigned long)data->imu_missed_reads,
        (uint64_t)data->last_hall_pulse_us
    );

    if (written >= (int)sizeof(json)) {
        ESP_LOGW(TAG, "Telemetry JSON truncated (%d >= %d)", written, (int)sizeof(json));
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t*)json;
    ws_pkt.len = strlen(json);

    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (client_fds[i] != 0) {
            esp_err_t ret = httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Failed to send telemetry to client %d (fd=%d)", i, client_fds[i]);
            }
        }
    }
    xSemaphoreGive(client_lock);
}

/**
 * @brief Send log message to all connected WebSocket clients
 */
#if ENABLE_WS_LOGS
static void send_log_to_clients(const log_message_t *log) {
    if (log == NULL) {
        ESP_LOGE(DIAG_TAG, "send_log_to_clients called with NULL log pointer");
        return;
    }
    if (client_count == 0) {
        return;  // No clients connected
    }

    char sanitized_msg[256];
    int j = 0;
    for (int i = 0; log->message[i] != '\0' && j < sizeof(sanitized_msg) - 1; i++) {
        if (log->message[i] == '\n' || log->message[i] == '\r') {
            sanitized_msg[j++] = ' ';
        } else {
            sanitized_msg[j++] = log->message[i];
        }
    }
    sanitized_msg[j] = '\0';

    char json[512];
    int written = snprintf(json, sizeof(json),
             "{\"l\":[%d,%llu,\"%s\",\"%s\"]}",
             log->level,
             log->timestamp_ms,
             log->source,
             sanitized_msg);

    if (written >= sizeof(json)) {
        ESP_LOGW(TAG, "Log JSON truncated (%d >= %d)", written, sizeof(json));
    }

    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    ws_pkt.payload = (uint8_t*)json;
    ws_pkt.len = strlen(json);

    xSemaphoreTake(client_lock, portMAX_DELAY);
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (client_fds[i] != 0) {
            esp_err_t ret = httpd_ws_send_frame_async(server, client_fds[i], &ws_pkt);
            if (ret != ESP_OK) {
                ESP_LOGD(TAG, "Failed to send log to client %d (fd=%d)", i, client_fds[i]);
            }
        }
    }
    xSemaphoreGive(client_lock);
}
#endif

/**
 * @brief WebSocket server task
 */
static void web_task(void *pvParameters) {
    telemetry_data_t telemetry;
    TickType_t last_telemetry_time = xTaskGetTickCount();
    TickType_t last_check_time = xTaskGetTickCount();
    
    while (1) {
        // Periodically check for stale clients
        if ((xTaskGetTickCount() - last_check_time) >= (CLIENT_PRUNE_INTERVAL_MS / portTICK_PERIOD_MS)) {
            check_and_remove_stale_clients();
            last_check_time = xTaskGetTickCount();
        }

        // Primary telemetry path now: control_task decimates 200Hz -> 10Hz and invokes telemetry callback,
        // which enqueues updates here (websocket_srv_publish_telemetry).
        telemetry_data_t *queue_data = NULL;
        if (xQueueReceive(telemetry_queue, &queue_data, 0) == pdTRUE) {
            send_telemetry_to_clients(queue_data);
            free(queue_data);
            last_telemetry_time = xTaskGetTickCount();
        } else {
            // Fallback: if no queued telemetry for >500 ms, send a snapshot so UI doesn't appear frozen.
            if ((xTaskGetTickCount() - last_telemetry_time) >= pdMS_TO_TICKS(500)) {
                if (client_count > 0 && controller_get_telemetry(&telemetry) == ESP_OK) {
                    send_telemetry_to_clients(&telemetry);
                }
                last_telemetry_time = xTaskGetTickCount();
            }
        }

#if ENABLE_WS_LOGS
        // Log forwarding (only when enabled)
        log_message_t *log_data = NULL;
        if (xQueueReceive(log_queue, &log_data, 0) == pdTRUE) {
            send_log_to_clients(log_data);
            free(log_data);
        }
#endif

        // Adaptive idle delay
        if (client_count == 0) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        } else {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

/**
 * @brief Start the HTTP server with WebSocket support
 */
static esp_err_t start_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    
    // Power-optimized server configuration
    config.max_open_sockets = MAX_WS_CLIENTS + 2;  // Reduce from 5 to save memory
    config.lru_purge_enable = true;        // Enable LRU purge for memory management
    config.task_priority = 4;              // Lower priority (default is 5)
    config.stack_size = 8192;              // Increased stack size for stability
    config.recv_wait_timeout = 10;         // Reduce timeout (default is 5)
    config.send_wait_timeout = 10;         // Reduce timeout (default is 5)
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d with power optimizations", config.server_port);
    
    if (httpd_start(&server, &config) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    // URI handler for root path
    httpd_uri_t index_uri = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = index_get_handler,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(server, &index_uri);
    
    // URI handler for WebSocket endpoint
    httpd_uri_t ws_uri = {
        .uri = "/ws",
        .method = HTTP_GET,
        .handler = ws_handler,
        .user_ctx = NULL,
        .is_websocket = true
    };
    httpd_register_uri_handler(server, &ws_uri);
    
    return ESP_OK;
}

/**
 * @brief Callback function for telemetry data from controller
 */
static esp_err_t telemetry_callback(const telemetry_data_t *data) {
    return websocket_srv_publish_telemetry(data);
}

esp_err_t websocket_srv_init(void) {
    // Create mutex for client list
    client_lock = xSemaphoreCreateMutex();
    if (client_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create client lock mutex");
        return ESP_FAIL;
    }
    
    // Create telemetry queue
    telemetry_queue = xQueueCreate(5, sizeof(telemetry_data_t*));
    if (telemetry_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create telemetry queue");
        vSemaphoreDelete(client_lock);
        return ESP_FAIL;
    }
    
#if ENABLE_WS_LOGS
    // Create log message queue (only when log forwarding enabled)
    log_queue = xQueueCreate(10, sizeof(log_message_t*));
    if (log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create log queue");
        vSemaphoreDelete(client_lock);
        vQueueDelete(telemetry_queue);
        return ESP_FAIL;
    }
#endif

    // Start the HTTP server
    esp_err_t ret = start_server();
    if (ret != ESP_OK) {
        vSemaphoreDelete(client_lock);
        vQueueDelete(telemetry_queue);
#if ENABLE_WS_LOGS
        vQueueDelete(log_queue);
#endif
        return ret;
    }

#if ENABLE_WS_LOGS
    // Register log callback to forward warnings/errors to browser
    esp_log_set_vprintf(websocket_log_vprintf);
#endif
    
    ESP_LOGI(TAG, "WebSocket server initialized successfully");
    return ESP_OK;
}

esp_err_t websocket_srv_start(void) {
    // Register our telemetry callback with the controller
    esp_err_t ret = controller_register_telemetry_callback(telemetry_callback);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to register telemetry callback, will continue anyway");
    }
    
    // Create the web task
    BaseType_t task_ret = xTaskCreatePinnedToCore(
        web_task,
        "web_task",
        WEB_TASK_STACK_SIZE,
        NULL,
        WEB_TASK_PRIORITY,
        &web_task_handle,
        1  // Pin to Core 1
    );
    
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create web task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "WebSocket server task started on Core 1");
    return ESP_OK;
}

esp_err_t websocket_srv_publish_telemetry(const telemetry_data_t *data) {
    if (data == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Don't allocate if there are no clients
    if (client_count == 0) {
        return ESP_OK;
    }
    
    // Allocate memory for the data to be queued
    telemetry_data_t *queue_data = malloc(sizeof(telemetry_data_t));
    if (queue_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for telemetry data");
        return ESP_ERR_NO_MEM;
    }
    
    // Copy the data
    memcpy(queue_data, data, sizeof(telemetry_data_t));
    
    // Send to queue
    if (xQueueSend(telemetry_queue, &queue_data, 0) != pdTRUE) {
        ESP_LOGW(DIAG_TAG, "Telemetry queue full, dropping telemetry update");
        free(queue_data);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

#if ENABLE_WS_LOGS
esp_err_t websocket_srv_publish_log(const log_message_t *log) {
    if (log == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (client_count == 0) {
        return ESP_OK;
    }
    log_message_t *queue_data = malloc(sizeof(log_message_t));
    if (queue_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for log data");
        return ESP_ERR_NO_MEM;
    }
    memcpy(queue_data, log, sizeof(log_message_t));
    if (xQueueSend(log_queue, &queue_data, 0) != pdTRUE) {
        free(queue_data);
        return ESP_FAIL;
    }
    return ESP_OK;
}
#else
esp_err_t websocket_srv_publish_log(const log_message_t *log) {
    (void)log;
    return ESP_OK; // No-op when websocket log forwarding disabled
}
#endif

#if ENABLE_WS_LOGS
// Custom vprintf to forward ESP-IDF logs to websocket_srv_publish_log
static int websocket_log_vprintf(const char *fmt, va_list args) {
   char buf[256];
   int len = vsnprintf(buf, sizeof(buf), fmt, args);
   if (len <= 0) return vprintf(fmt, args);

   char level_char = buf[0];
   log_level_t level = LOG_LEVEL_INFO;
   switch (level_char) {
       case 'E': level = LOG_LEVEL_ERROR; break;
       case 'W': level = LOG_LEVEL_WARNING; break;
       case 'I': level = LOG_LEVEL_INFO; break;
       case 'D': level = LOG_LEVEL_DEBUG; break;
       default:  level = LOG_LEVEL_INFO; break;
   }
   if (level < WS_LOG_MIN_LEVEL) return vprintf(fmt, args);

   const char *tag_start = strchr(buf, ')');
   const char *msg_start = 0;
   char tag[32] = {0};
   if (tag_start) {
       const char *tag_colon = strchr(tag_start, ':');
       if (tag_colon && tag_colon > tag_start + 2) {
           size_t tag_len = tag_colon - (tag_start + 2);
           if (tag_len < sizeof(tag)) {
               strncpy(tag, tag_start + 2, tag_len);
               tag[tag_len] = '\0';
           }
           msg_start = tag_colon + 2;
       } else {
           msg_start = tag_start + 1;
       }
   } else {
       msg_start = buf;
   }

   log_message_t log_msg = {0};
   log_msg.level = level;
   log_msg.timestamp_ms = esp_log_timestamp();
   strncpy(log_msg.message, msg_start, sizeof(log_msg.message) - 1);
   strncpy(log_msg.source, tag, sizeof(log_msg.source) - 1);
   websocket_srv_publish_log(&log_msg);

   return vprintf(fmt, args);
}
#endif

