/**
 * @file adxl345.c
 * @brief Implementação do driver ADXL345 para ESP-IDF v6.x (I2C Master Moderno)
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdlib.h>
#include <string.h>
#include "adxl345.h"
#include "esp_log.h"
#include "esp_check.h"

static const char *TAG = "ADXL345";

/* Registradores do ADXL345 */
#define REG_DEVID          0x00
#define REG_BW_RATE        0x2C
#define REG_POWER_CTL      0x2D
#define REG_INT_ENABLE     0x2E
#define REG_DATA_FORMAT    0x31
#define REG_DATAX0        0x32

#define EXPECTED_DEVID     0xE5
#define FULL_RES_BIT       0x08
#define MEASURE_BIT        0x08

#define SCALE_FACTOR_G     0.0039f      /* 3.9 mg/LSB no modo FULL_RES */
#define CONST_GRAVITY      9.80665f

struct adxl345_dev_t {
    i2c_master_dev_handle_t i2c_dev;
};

static esp_err_t adxl345_write_reg(adxl345_handle_t handle, uint8_t reg_addr, uint8_t value)
{
    uint8_t write_buf[2] = {reg_addr, value};
    return i2c_master_transmit(handle->i2c_dev, write_buf, sizeof(write_buf), pdMS_TO_TICKS(100));
}

static esp_err_t adxl345_read_regs(adxl345_handle_t handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(handle->i2c_dev, &reg_addr, 1, data, len, pdMS_TO_TICKS(100));
}

esp_err_t adxl345_init(const adxl345_config_t *config, adxl345_handle_t *ret_handle)
{
    ESP_RETURN_ON_FALSE(config && ret_handle && config->bus_handle, ESP_ERR_INVALID_ARG, TAG, "Argumentos invalidos");

    adxl345_handle_t dev = (adxl345_handle_t)malloc(sizeof(struct adxl345_dev_t));
    if (!dev) {
        return ESP_ERR_NO_MEM;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = config->dev_addr ? config->dev_addr : ADXL345_I2C_ADDR_ALT_LOW,
        .scl_speed_hz    = config->scl_speed_hz ? config->scl_speed_hz : 400000,
    };

    esp_err_t ret = i2c_master_bus_add_device(config->bus_handle, &dev_cfg, &dev->i2c_dev);
    if (ret != ESP_OK) {
        free(dev);
        ESP_LOGE(TAG, "Falha ao anexar dispositivo ao barramento I2C");
        return ret;
    }

    // Identificação do sensor
    uint8_t dev_id = 0;
    ret = adxl345_read_regs(dev, REG_DEVID, &dev_id, 1);
    if (ret != ESP_OK || dev_id != EXPECTED_DEVID) {
        ESP_LOGE(TAG, "ADXL345 nao encontrado! ID lido: 0x%02X (esperado: 0x%02X)", dev_id, EXPECTED_DEVID);
        i2c_master_bus_rm_device(dev->i2c_dev);
        free(dev);
        return (ret != ESP_OK) ? ret : ESP_ERR_NOT_FOUND;
    }

    // Configuração de taxa e formato (Full-Res ativado)
    ret = adxl345_set_datarate(dev, config->datarate);
    if (ret != ESP_OK) goto err;

    ret = adxl345_set_range(dev, config->range);
    if (ret != ESP_OK) goto err;

    // Habilita modo de medição contínua
    ret = adxl345_write_reg(dev, REG_POWER_CTL, MEASURE_BIT);
    if (ret != ESP_OK) goto err;

    *ret_handle = dev;
    ESP_LOGI(TAG, "ADXL345 inicializado com sucesso.");
    return ESP_OK;

err:
    i2c_master_bus_rm_device(dev->i2c_dev);
    free(dev);
    return ret;
}

esp_err_t adxl345_set_datarate(adxl345_handle_t handle, adxl345_datarate_t datarate)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle nulo");
    return adxl345_write_reg(handle, REG_BW_RATE, (uint8_t)datarate);
}

esp_err_t adxl345_set_range(adxl345_handle_t handle, adxl345_range_t range)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle nulo");
    // Mantém o bit FULL_RES (0x08) sempre ativado para resolução de 3.9 mg/LSB em todas as escalas
    uint8_t format_val = FULL_RES_BIT | ((uint8_t)range & 0x03);
    return adxl345_write_reg(handle, REG_DATA_FORMAT, format_val);
}

esp_err_t adxl345_get_raw(adxl345_handle_t handle, adxl345_raw_data_t *raw_data)
{
    ESP_RETURN_ON_FALSE(handle && raw_data, ESP_ERR_INVALID_ARG, TAG, "Parametros invalidos");

    uint8_t buffer[6];
    esp_err_t ret = adxl345_read_regs(handle, REG_DATAX0, buffer, sizeof(buffer));
    if (ret != ESP_OK) {
        return ret;
    }

    raw_data->x = (int16_t)((buffer[1] << 8) | buffer[0]);
    raw_data->y = (int16_t)((buffer[3] << 8) | buffer[2]);
    raw_data->z = (int16_t)((buffer[5] << 8) | buffer[4]);

    return ESP_OK;
}

esp_err_t adxl345_get_accel_g(adxl345_handle_t handle, adxl345_data_t *data)
{
    ESP_RETURN_ON_FALSE(handle && data, ESP_ERR_INVALID_ARG, TAG, "Parametros invalidos");

    adxl345_raw_data_t raw;
    esp_err_t ret = adxl345_get_raw(handle, &raw);
    if (ret != ESP_OK) {
        return ret;
    }

    data->x = (float)raw.x * SCALE_FACTOR_G;
    data->y = (float)raw.y * SCALE_FACTOR_G;
    data->z = (float)raw.z * SCALE_FACTOR_G;

    return ESP_OK;
}

esp_err_t adxl345_get_accel_ms2(adxl345_handle_t handle, adxl345_data_t *data)
{
    esp_err_t ret = adxl345_get_accel_g(handle, data);
    if (ret == ESP_OK) {
        data->x *= CONST_GRAVITY;
        data->y *= CONST_GRAVITY;
        data->z *= CONST_GRAVITY;
    }
    return ret;
}

esp_err_t adxl345_deinit(adxl345_handle_t handle)
{
    ESP_RETURN_ON_FALSE(handle, ESP_ERR_INVALID_ARG, TAG, "Handle nulo");

    // Coloca o sensor em standby
    adxl345_write_reg(handle, REG_POWER_CTL, 0x00);

    esp_err_t ret = i2c_master_bus_rm_device(handle->i2c_dev);
    free(handle);
    return ret;
}
