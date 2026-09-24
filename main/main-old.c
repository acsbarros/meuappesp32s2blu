#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <ssd1306.h>
#include "dht11.h"
#include "sdkconfig.h"

//wifi
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"


// Modern ADC (ESP-IDF v5.x+)
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "inttypes.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_system.h"

// ==================== DHT11 ====================
#define CONFIG_DHT11_PIN          15
#define CONFIG_CONNECTION_TIMEOUT 5

// ==================== UART / Bluetooth ====================
#define BT_UART_NUM  UART_NUM_1
#define TXD_PIN      17
#define RXD_PIN      18
#define BUF_SIZE     (1024)
#define LED_GPIO     13

// ==================== Tasks ====================
#define SENSOR_TASK_STACK       2048
#define DISPLAY_TASK_STACK      4096
#define BT_SEND_TASK_STACK      3072
#define MONITOR_TASK_STACK      2048

#define SENSOR_TASK_PRIO        5
#define DISPLAY_TASK_PRIO       4
#define BT_SEND_TASK_PRIO       3
#define MONITOR_TASK_PRIO       1
#define SENSOR_READ_INTERVAL_MS 2000

// ==================== PWM ====================
#define LED_PWM_GPIO        2
#define LEDC_MODE           LEDC_LOW_SPEED_MODE
#define LEDC_TIMER          LEDC_TIMER_0
#define LEDC_CHANNEL        LEDC_CHANNEL_0
#define LEDC_DUTY_RES       LEDC_TIMER_14_BIT
#define LEDC_FREQUENCY      50

// ==================== Interrupção ====================
#define INTERRUPT_PIN       5

static const char *TAG = "UART_BT_APP";

int adc_value = 0;

// Estrutura compartilhada entre dht11_task e oled_task
typedef struct {
    uint8_t temperature;  // °C
    uint8_t humidity;     // %
} dht11_data_t;

// Filas
static QueueHandle_t dht11_queue = NULL;
static QueueHandle_t bt_data_queue = NULL;
static QueueHandle_t adc_queue = NULL;
static QueueHandle_t gpio_evt_queue = NULL;

// Protótipo antecipado da ISR
static void gpio_isr_handler(void *arg);

// Tempos de pulso padrão para servos (em microssegundos)
// Ajuste esses valores caso o seu servo não chegue aos 180 graus exatos
#define SERVO_MIN_PULSEWIDTH_US      500      // Equivalente a 0 graus
#define SERVO_MAX_PULSEWIDTH_US      2500     // Equivalente a 180 graus



// ============================================================
// WIFI
// ============================================================
void wifi_init_sta(void)
{
    // Cria a interface padrão do tipo Station
    esp_netif_create_default_wifi_sta();

    // Inicializa o driver Wi-Fi com as configurações padrão
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "NOME_DA_SUA_REDE",
            .password = "SENHA_DA_SUA_REDE",
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
}


// ============================================================
// PWM
// ============================================================

// Função para calcular e aplicar o duty cycle baseado no ângulo desejado (0 a 180)
void set_servo_angle(int angle) {
    // Garante que o ângulo esteja nos limites
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // Calcula o pulso em microssegundos para o ângulo atual
    uint32_t pulsewidth_us = SERVO_MIN_PULSEWIDTH_US + 
        (((SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US) * angle) / 180);

    // Converte o tempo de pulso para duty cycle (para 14 bits e 50Hz)
    // Fórmula: (pulsewidth_us / 20000_us) * (2^14 - 1)
    uint32_t duty = (pulsewidth_us * ((1 << LEDC_DUTY_RES) - 1)) / (1000000 / LEDC_FREQUENCY);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
}

static void pwm_servo_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando ciclo PWM do servo...");

    while (1) {
        ESP_LOGI(TAG, "Indo para 0 graus");
        set_servo_angle(0);
        vTaskDelay(pdMS_TO_TICKS(1000));

        
        ESP_LOGI(TAG, "Indo para 90 graus");
        set_servo_angle(90);
        vTaskDelay(pdMS_TO_TICKS(1000));

        ESP_LOGI(TAG, "Indo para 180 graus");
        set_servo_angle(180);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}



static void pwm_fade_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Iniciando ciclo PWM do led...");

    while (1) {
        // Aumenta o brilho gradativamente (0 a 255)
        for (int i = 0; i <= 255; i++) {
            ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, i, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        // Diminui o brilho gradativamente (255 a 0)
        for (int i = 255; i >= 0; i--) {
            ledc_set_duty_and_update(LEDC_MODE, LEDC_CHANNEL, i, 0);
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    vTaskDelete(NULL);
}

void initpwd(void)
{
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num       = LEDC_TIMER,
        .freq_hz         = LEDC_FREQUENCY,
        .clk_cfg         = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer));

    ledc_channel_config_t channel = {
        .gpio_num   = LED_PWM_GPIO,
        .speed_mode = LEDC_MODE,
        .channel    = LEDC_CHANNEL,
        .timer_sel  = LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel));
    ESP_ERROR_CHECK(ledc_fade_func_install(0));
    ESP_LOGI(TAG,"Iniciando controle do Servo Motor no pino %d...\n", LED_PWM_GPIO);


}

// ============================================================
// LED
// ============================================================
static void init_led(void)
{
    gpio_reset_pin(LED_GPIO);
    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);
    ESP_LOGI(TAG, "LED inicializado no GPIO %d", LED_GPIO);
}

// ============================================================
// UART
// ============================================================
void init_uart(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = 9600,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(BT_UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(BT_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(BT_UART_NUM, TXD_PIN, RXD_PIN,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART1 inicializada no BaudRate 9600!");
}

// ============================================================
// INTERRUPÇÃO
// ============================================================
static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    xQueueSendFromISR(gpio_evt_queue, &gpio_num, &xHigherPriorityTaskWoken);

    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void init_interrupt(void)
{
    gpio_reset_pin(INTERRUPT_PIN);
    gpio_set_direction(INTERRUPT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(INTERRUPT_PIN, GPIO_PULLUP_ONLY);
    gpio_set_intr_type(INTERRUPT_PIN, GPIO_INTR_POSEDGE);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(INTERRUPT_PIN, gpio_isr_handler, (void *) INTERRUPT_PIN);
    gpio_intr_enable(INTERRUPT_PIN);

    ESP_LOGI(TAG, "Tratamento por interrupção inicializado com sucesso.");
}

static void button_task(void *pvParameters)
{
    uint32_t io_num;
    uint32_t count = 0;
    bool led_state = false;

    while (1) {
        if (xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            count++;
            led_state = !led_state;
            gpio_set_level(LED_GPIO, led_state);

            ESP_LOGI(TAG, "Interrupção confirmada no GPIO[%" PRIu32 "] | Contagem: %" PRIu32 " | LED: %s",
                     io_num, count, led_state ? "LIGADO" : "DESLIGADO");
        }
    }
}

// ============================================================
// TASK 1 — Recebe dados do módulo Bluetooth (LED)
// ============================================================
static void rx_task(void *arg)
{
    uint8_t *data = (uint8_t *) malloc(BUF_SIZE + 1);

    // Variável para guardar o ângulo atual. Inicia no centro (90 graus)
    static int current_servo_angle = 90;
    // Move o servo para o centro ao iniciar a task
    set_servo_angle(current_servo_angle);
    ESP_LOGI(TAG, "Servo posicionado no centro (90 graus)");

    while (1) {
        const int rxBytes = uart_read_bytes(BT_UART_NUM, data, BUF_SIZE, pdMS_TO_TICKS(100));
        if (rxBytes > 0) {
            data[rxBytes] = '\0';
            ESP_LOGI(TAG, "Recebido: %s", (char*)data);

            for (int i = 0; i < rxBytes; i++) {
                if (data[i] == '1') {
                    gpio_set_level(LED_GPIO, 1);
                    ESP_LOGI(TAG, "LED -> LIGADO");
                    const char *resp = "LED_ON\n";
                    uart_write_bytes(BT_UART_NUM, resp, strlen(resp));
                } else if (data[i] == '0') {
                    gpio_set_level(LED_GPIO, 0);
                    ESP_LOGI(TAG, "LED -> DESLIGADO");
                    const char *resp = "LED_OFF\n";
                    uart_write_bytes(BT_UART_NUM, resp, strlen(resp));
                }else if (data[i] == '2') {
                    //pwm
                    // DIREITA: Incrementa em 1 grau (limite máximo de 180)
                    if (current_servo_angle < 180) {
                        current_servo_angle=current_servo_angle+10;
                    }
                    set_servo_angle(current_servo_angle);
                    
                    ESP_LOGI(TAG, "DIREITA -> Angulo atual: %d", current_servo_angle);
                    char resp[20];
                    snprintf(resp, sizeof(resp), "DIR:%d\n", current_servo_angle);
                    uart_write_bytes(BT_UART_NUM, resp, strlen(resp));
                }
                else if (data[i] == '3') {
                    //pwm
                    if (current_servo_angle > 0) {
                        current_servo_angle=current_servo_angle-10;
                    }
                    set_servo_angle(current_servo_angle);
                    
                    ESP_LOGI(TAG, "ESQUERDA -> Angulo atual: %d", current_servo_angle);
                    char resp[20];
                    snprintf(resp, sizeof(resp), "ESQ:%d\n", current_servo_angle);
                    uart_write_bytes(BT_UART_NUM, resp, strlen(resp));
                }
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}

// ============================================================
// TASK 2 — Leitura do DHT11
// ============================================================
static void dht11_task(void *pvParameters)
{
    dht11_t dht11_sensor;
    dht11_sensor.dht11_pin = CONFIG_DHT11_PIN;

    dht11_data_t data;
    uint8_t prev_temp = 0xFF;
    uint8_t prev_hum  = 0xFF;

    ESP_LOGI(TAG, "Task DHT11 iniciada no GPIO %d", CONFIG_DHT11_PIN);

    while (1) {
        if (!dht11_read(&dht11_sensor, CONFIG_CONNECTION_TIMEOUT)) {
            uint8_t curr_temp = (uint8_t)dht11_sensor.temperature;
            uint8_t curr_hum  = (uint8_t)dht11_sensor.humidity;

            if (curr_temp != prev_temp || curr_hum != prev_hum) {
                data.temperature = curr_temp;
                data.humidity    = curr_hum;

                if (xQueueSend(dht11_queue, &data, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Fila DHT11 cheia, leitura descartada");
                }

                if (xQueueSend(bt_data_queue, &data, 0) != pdTRUE) {
                    ESP_LOGW(TAG, "Fila BT cheia, leitura descartada");
                }

                prev_temp = curr_temp;
                prev_hum  = curr_hum;

                ESP_LOGI(TAG, "Temp: %dC | Hum: %d%%", curr_temp, curr_hum);
            }
        } else {
            ESP_LOGW(TAG, "Falha ao ler o DHT11");
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}

// ============================================================
// TASK 3 — Escrita no display OLED
// ============================================================
static void oled_task(void *pvParameters)
{
    dht11_data_t data;
    char temp_str[20];
    char humidity_str[25];

    ESP_LOGI(TAG, "Task OLED iniciada");

    ssd1306_clear();
    ssd1306_print_str(20, 18, "Aguardando...", false);
    ssd1306_display();

    while (1) {
        if (xQueueReceive(dht11_queue, &data, portMAX_DELAY) == pdTRUE) {
            snprintf(temp_str, sizeof(temp_str), "Temp: %dC", data.temperature);
            snprintf(humidity_str, sizeof(humidity_str), "Humidity: %d%%", data.humidity);

            ssd1306_clear();
            ssd1306_print_str(25, 18, temp_str, false);
            ssd1306_print_str(8, 28, humidity_str, false);
            ssd1306_display();

            ESP_LOGI(TAG, "Display atualizado: %s | %s", temp_str, humidity_str);
        }
    }
}

// ============================================================
// TASK 4 — Envia temperatura/umidade via Bluetooth (UART)
// ============================================================
static void bt_send_task(void *pvParameters)
{
    dht11_data_t data;
    char msg[32];

    ESP_LOGI(TAG, "Task BT Send iniciada");

    while (1) {
        if (xQueueReceive(bt_data_queue, &data, portMAX_DELAY) == pdTRUE) {
            int len = snprintf(msg, sizeof(msg), "T:%d;H:%d\n", data.temperature, data.humidity);
            uart_write_bytes(BT_UART_NUM, msg, len);
            ESP_LOGI(TAG, "BT enviado: %s", msg);
        }
    }
}
/*
// ============================================================
// TASK 5 — Leitura do ADC (ESP-IDF v5.x - OneShot API)
// ============================================================
void adc_read_task(void *pvParameters)
{
    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_config_t init_config = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc1_handle));

    adc_oneshot_chan_config_t chan_config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten    = ADC_ATTEN_DB_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &chan_config));

    ESP_LOGI(TAG, "ADC Oneshot inicializado no ADC1 Canal 0. Iniciando leituras...");

    while (1) {
        ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adc_value));
        xQueueSend(adc_queue, &adc_value, pdMS_TO_TICKS(10));
        ESP_LOGI(TAG, "ADC Value is %d", adc_value);

        vTaskDelay(pdMS_TO_TICKS(100));
    }

    adc_oneshot_del_unit(adc1_handle);
    vTaskDelete(NULL);
}

// ============================================================
// TASK 6 — Consumidora do ADC
// ============================================================
void data_processing_task(void *pvParameters)
{
    int valor_recebido = 0;

    while (1) {
        if (xQueueReceive(adc_queue, &valor_recebido, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG, "Dado retirado da fila para processamento: %d", valor_recebido);
        }
    }
}
*/
// ============================================================
// TASK NOVA (TASK 7) — Monitoramento de Recursos e Heap
// ============================================================
static void system_monitor_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Task de monitoramento do sistema iniciada.");

    while (1) {
        uint32_t free_heap = esp_get_free_heap_size();
        uint32_t min_free_heap = esp_get_minimum_free_heap_size();

        ESP_LOGI("SYS_MONITOR", "Heap Livre: %" PRIu32 " bytes | Min Heap Histórico: %" PRIu32 " bytes",
                 free_heap, min_free_heap);

        // Se a memória cair abaixo de 20KB, gera um alerta preventivo
        if (free_heap < 20480) {
            ESP_LOGW("SYS_MONITOR", "Alerta: Pouca memória RAM livre no sistema!");
        }

        vTaskDelay(pdMS_TO_TICKS(5000)); // Verifica a cada 5 segundos
    }

    vTaskDelete(NULL);
}

// ============================================================
// app_main
// ============================================================
void app_main(void)
{
    ESP_LOGI(TAG, "Iniciando sistema...");

    #if CONFIG_FREQUENCIA_DE_INICIALIZACAO
        printf("LED configurado no pino GPIO: %d\n", CONFIG_MY_LED_GPIO);
    #else
        printf("Funcionalidade desabilitada no Kconfig.\n");
    #endif

    // ---- Periféricos ----
    init_ssd1306();
    init_led();
    init_uart();
    initpwd();
    init_interrupt();

    // ---- Filas compartilhadas ----
    dht11_queue    = xQueueCreate(5, sizeof(dht11_data_t));
    bt_data_queue  = xQueueCreate(5, sizeof(dht11_data_t));
    adc_queue      = xQueueCreate(10, sizeof(int));
    gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    if (dht11_queue == NULL || bt_data_queue == NULL || adc_queue == NULL || gpio_evt_queue == NULL) {
        ESP_LOGE(TAG, "Falha ao criar as filas FreeRTOS. Abortando inicialização.");
        return;
    }

    // ---- Cria as tarefas ----
    xTaskCreate(rx_task,              "uart_rx_task",   4096,                NULL, 5,                  NULL);
    xTaskCreate(oled_task,            "oled_task",      DISPLAY_TASK_STACK,  NULL, DISPLAY_TASK_PRIO,  NULL);
    xTaskCreate(dht11_task,           "dht11_task",     SENSOR_TASK_STACK,   NULL, SENSOR_TASK_PRIO,   NULL);
    xTaskCreate(bt_send_task,         "bt_send_task",   BT_SEND_TASK_STACK,  NULL, BT_SEND_TASK_PRIO,  NULL);
    //xTaskCreate(adc_read_task,        "adc_read_task",  2048,                NULL, 5,                  NULL);
    //xTaskCreate(data_processing_task, "proc_task",      2048,                NULL, 4,                  NULL);
    xTaskCreate(button_task,          "button_task",    2048,                NULL, 10,                 NULL);
    //xTaskCreate(pwm_fade_task,        "pwm_fade_task",  2048,                NULL, 5,                  NULL);
    //xTaskCreate(pwm_servo_task,        "pwm_servo_task",  2048,                NULL, 5,                  NULL);

    // Criação da nova task de monitoramento
    xTaskCreate(system_monitor_task,  "sys_monitor",    MONITOR_TASK_STACK,  NULL, MONITOR_TASK_PRIO,  NULL);

    const char *msg = "Franzininho WiFi conectada via ESP-IDF!\r\n";
    uart_write_bytes(BT_UART_NUM, msg, strlen(msg));

    ESP_LOGI(TAG, "Sistema pronto — Todas as tarefas inicializadas com sucesso!");
}