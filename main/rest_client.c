#include "rest_client.h"
#include "esp_log.h"
#include <string.h>

#include <stdio.h>
#include "cJSON.h"

static const char *TAG = "JSON_APP";

typedef struct {
    char *buf;
    size_t size;
    size_t len;
} http_buffer_t;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        http_buffer_t *resp = (http_buffer_t *)evt->user_data;
        if (resp && resp->buf && (resp->len + evt->data_len < resp->size - 1)) {
            memcpy(resp->buf + resp->len, evt->data, evt->data_len);
            resp->len += evt->data_len;
            resp->buf[resp->len] = '\0';
        }
    }
    return ESP_OK;
}

esp_err_t rest_request(
    esp_http_client_method_t method,
    const char *url,
    const char *token,
    const char *payload,
    char *response_buf,
    size_t response_size
) {
    http_buffer_t resp_data = { .buf = response_buf, .size = response_size, .len = 0 };
    if (response_buf && response_size > 0) response_buf[0] = '\0';

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .user_data = &resp_data,
        .timeout_ms = 8000,
    };

    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_method(client, method);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    if (token && strlen(token) > 0) {
        char auth_hdr[300];
        snprintf(auth_hdr, sizeof(auth_hdr), "Bearer %s", token);
        esp_http_client_set_header(client, "Authorization", auth_hdr);
    }

    if (payload && (method == HTTP_METHOD_POST || method == HTTP_METHOD_PUT || method == HTTP_METHOD_PATCH)) {
        esp_http_client_set_post_field(client, payload, strlen(payload));
    }

    esp_err_t err = esp_http_client_perform(client);
    int status_code = esp_http_client_get_status_code(client);
    
    ESP_LOGI(TAG, "HTTP Method: %d | Status: %d", method, status_code);
    esp_http_client_cleanup(client);

    return (err == ESP_OK && status_code >= 200 && status_code < 300) ? ESP_OK : ESP_FAIL;
}


/*
static const char *TAG = "MAIN_APP";

// --- EXEMPLO 1: Construir payload JSON para enviar no POST/PUT ---
void enviar_dados_sensor(const char *token)
{
    // 1. Criar o objeto JSON raiz
    cJSON *root = cJSON_CreateObject();
    if (root == NULL) {
        ESP_LOGE(TAG, "Falha ao alocar memoria para o JSON");
        return;
    }

    // 2. Adicionar campos ao objeto
    cJSON_AddStringToObject(root, "dispositivo", "ESP32-S2");
    cJSON_AddNumberToObject(root, "temperatura", 26.8);
    cJSON_AddNumberToObject(root, "umidade", 55.4);
    cJSON_AddBoolToObject(root, "status_ok", true);

    // Adicionar um sub-objeto ou array (exemplo de metadados)
    cJSON *sub_obj = cJSON_CreateObject();
    cJSON_AddStringToObject(sub_obj, "local", "Laboratorio");
    cJSON_AddItemToObject(root, "meta", sub_obj);

    // 3. Converter a estrutura cJSON para string formatada
    char *json_string = cJSON_PrintUnformatted(root);

    if (json_string != NULL) {
        printf("JSON gerado para envio: %s\n", json_string);

        char response_buffer[1024];
        esp_err_t err = rest_post("https://api.exemplo.com/v1/telemetria", 
                                  token, 
                                  json_string, 
                                  response_buffer, 
                                  sizeof(response_buffer));

        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Post realizado com sucesso!");
        }

        // IMPORTANTE: Liberar a memória da string gerada pelo cJSON_Print
        cJSON_free(json_string);
    }

    // 4. Liberar a memória da árvore de objetos cJSON
    cJSON_Delete(root);
}


// --- EXEMPLO 2: Fazer GET e analisar (parse) o JSON retornado ---
void ler_configuracao_remota(const char *token)
{
    char response_buffer[2048];

    esp_err_t err = rest_get("https://api.exemplo.com/v1/configuracao", 
                             token, 
                             response_buffer, 
                             sizeof(response_buffer));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Falha ao fazer requisição GET");
        return;
    }

    // Exemplo de resposta esperada do servidor:
    // {
    //   "intervalo_envio": 30,
    //   "servidor_ativo": true,
    //   "firmware_versao": "v2.1.0",
    //   "alarmes": ["alta_temp", "bateria_baixa"]
    // }

    // 1. Converter a string da resposta para objeto cJSON
    cJSON *json = cJSON_Parse(response_buffer);
    if (json == NULL) {
        const char *error_ptr = cJSON_GetErrorPtr();
        if (error_ptr != NULL) {
            ESP_LOGE(TAG, "Erro de sintaxe no JSON antes de: %s", error_ptr);
        }
        return;
    }

    // 2. Extrair campos do tipo Número
    cJSON *intervalo = cJSON_GetObjectItemCaseSensitive(json, "intervalo_envio");
    if (cJSON_IsNumber(intervalo)) {
        printf("Intervalo de envio: %d segundos\n", intervalo->valueint);
    }

    // 3. Extrair campos do tipo Boolean
    cJSON *ativo = cJSON_GetObjectItemCaseSensitive(json, "servidor_ativo");
    if (cJSON_IsBool(ativo)) {
        printf("Servidor Ativo: %s\n", cJSON_IsTrue(ativo) ? "Sim" : "Nao");
    }

    // 4. Extrair campos do tipo String
    cJSON *versao = cJSON_GetObjectItemCaseSensitive(json, "firmware_versao");
    if (cJSON_IsString(versao) && (versao->valuestring != NULL)) {
        printf("Versão do Firmware: %s\n", versao->valuestring);
    }

    // 5. Extrair elementos de um Array
    cJSON *alarmes = cJSON_GetObjectItemCaseSensitive(json, "alarmes");
    if (cJSON_IsArray(alarmes)) {
        cJSON *alarme = NULL;
        printf("Alarmes configurados:\n");
        cJSON_ArrayForEach(alarme, alarmes) {
            if (cJSON_IsString(alarme)) {
                printf(" - %s\n", alarme->valuestring);
            }
        }
    }

    // 6. Liberar a memória da estrutura analisada
    cJSON_Delete(json);
}

// Lembre-se de inicializar a NVS e conectar ao Wi-Fi antes de chamar as funções
    const char *token = "seu_bearer_token_aqui";

    ler_configuracao_remota(token);
    enviar_dados_sensor(token);

*/
