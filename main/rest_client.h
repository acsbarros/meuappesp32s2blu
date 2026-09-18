#ifndef REST_CLIENT_H
#define REST_CLIENT_H

#include "esp_err.h"
#include "esp_http_client.h"

esp_err_t rest_request(
    esp_http_client_method_t method,
    const char *url,
    const char *token,
    const char *payload,
    char *response_buf,
    size_t response_size
);

// Wrappers simplificados
#define rest_get(url, token, resp, size) \
    rest_request(HTTP_METHOD_GET, url, token, NULL, resp, size)

#define rest_post(url, token, payload, resp, size) \
    rest_request(HTTP_METHOD_POST, url, token, payload, resp, size)

#define rest_put(url, token, payload, resp, size) \
    rest_request(HTTP_METHOD_PUT, url, token, payload, resp, size)

#define rest_delete(url, token, resp, size) \
    rest_request(HTTP_METHOD_DELETE, url, token, NULL, resp, size)

#endif