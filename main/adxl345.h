/**
 * @file adxl345.h
 * @brief Driver para acelerômetro triaxial ADXL345 (ESP-IDF v6.x / I2C Master Moderno)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Endereços I2C */
#define ADXL345_I2C_ADDR_ALT_LOW    0x53  /*!< SDO/ALT ADDRESS em GND (padrão) */
#define ADXL345_I2C_ADDR_ALT_HIGH   0x1D  /*!< SDO/ALT ADDRESS em VDD */

/* Faixas de medição (Range) */
typedef enum {
    ADXL345_RANGE_2G  = 0x00,
    ADXL345_RANGE_4G  = 0x01,
    ADXL345_RANGE_8G  = 0x02,
    ADXL345_RANGE_16G = 0x03,
} adxl345_range_t;

/* Taxas de amostragem de dados (Output Data Rate) */
typedef enum {
    ADXL345_DATARATE_0_10_HZ = 0x00,
    ADXL345_DATARATE_0_20_HZ = 0x01,
    ADXL345_DATARATE_0_39_HZ = 0x02,
    ADXL345_DATARATE_0_78_HZ = 0x03,
    ADXL345_DATARATE_1_56_HZ = 0x04,
    ADXL345_DATARATE_3_13_HZ = 0x05,
    ADXL345_DATARATE_6_25_HZ = 0x06,
    ADXL345_DATARATE_12_5_HZ = 0x07,
    ADXL345_DATARATE_25_HZ   = 0x08,
    ADXL345_DATARATE_50_HZ   = 0x09,
    ADXL345_DATARATE_100_HZ  = 0x0A,
    ADXL345_DATARATE_200_HZ  = 0x0B,
    ADXL345_DATARATE_400_HZ  = 0x0C,
    ADXL345_DATARATE_800_HZ  = 0x0D,
    ADXL345_DATARATE_1600_HZ = 0x0E,
    ADXL345_DATARATE_3200_HZ = 0x0F,
} adxl345_datarate_t;

/* Estrutura de dados crus (inteiros com sinal) */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} adxl345_raw_data_t;

/* Estrutura de aceleração convertida (m/s² ou g) */
typedef struct {
    float x;
    float y;
    float z;
} adxl345_data_t;

/* Estrutura de configuração para inicialização */
typedef struct {
    i2c_master_bus_handle_t bus_handle;   /*!< Handle do barramento I2C compartilhado */
    uint16_t dev_addr;                    /*!< Endereço I2C (0x53 ou 0x1D) */
    uint32_t scl_speed_hz;                /*!< Frequência do I2C (ex: 400000) */
    adxl345_range_t range;                /*!< Escala selecionada */
    adxl345_datarate_t datarate;          /*!< Taxa de saída de dados */
} adxl345_config_t;

/* Tipo opaco para o descritor do dispositivo ADXL345 */
typedef struct adxl345_dev_t *adxl345_handle_t;

/**
 * @brief Inicializa e configura o sensor ADXL345 no barramento I2C especificado.
 *
 * @param config Ponteiro para a estrutura de configuração do ADXL345.
 * @param ret_handle Ponteiro de saída para o descritor criado.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_init(const adxl345_config_t *config, adxl345_handle_t *ret_handle);

/**
 * @brief Lê os valores crus de aceleração (16-bit com sinal).
 *
 * @param handle Descritor do ADXL345.
 * @param raw_data Ponteiro de saída para os valores crus.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_get_raw(adxl345_handle_t handle, adxl345_raw_data_t *raw_data);

/**
 * @brief Lê os valores de aceleração em gravidades (g).
 *
 * @param handle Descritor do ADXL345.
 * @param data Ponteiro de saída para a aceleração em g.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_get_accel_g(adxl345_handle_t handle, adxl345_data_t *data);

/**
 * @brief Lê os valores de aceleração em metros por segundo ao quadrado (m/s²).
 *
 * @param handle Descritor do ADXL345.
 * @param data Ponteiro de saída para a aceleração em m/s².
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_get_accel_ms2(adxl345_handle_t handle, adxl345_data_t *data);

/**
 * @brief Altera a taxa de amostragem (Output Data Rate).
 *
 * @param handle Descritor do ADXL345.
 * @param datarate Nova taxa desejada.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_set_datarate(adxl345_handle_t handle, adxl345_datarate_t datarate);

/**
 * @brief Altera a faixa de medição (Range).
 *
 * @param handle Descritor do ADXL345.
 * @param range Nova faixa desejada (2G, 4G, 8G, 16G).
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_set_range(adxl345_handle_t handle, adxl345_range_t range);

/**
 * @brief Libera os recursos alocados pelo ADXL345 e o remove do barramento I2C.
 *
 * @param handle Descritor do ADXL345.
 * @return ESP_OK em caso de sucesso.
 */
esp_err_t adxl345_deinit(adxl345_handle_t handle);

#ifdef __cplusplus
}
#endif
