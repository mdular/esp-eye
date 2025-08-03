#ifndef HTTP_SERVER_H
#define HTTP_SERVER_H

/**
 * Start the HTTP server
 */
void start_http_server(void);

/**
 * Update data in WebSocket connections
 * @param counter The counter value to send to WebSocket clients
 */
void update_websocket_data(int counter);

#endif // HTTP_SERVER_H
