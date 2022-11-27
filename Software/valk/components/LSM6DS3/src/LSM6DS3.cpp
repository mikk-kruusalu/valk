#include "LSM6DS3.h"
#include "lsm6ds3tr-c_reg.h"
#include "CPPI2C/cppi2c.h"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "LSM6DS3";

/**
 * @brief Write generic device register
 *  
 * @param dev       customizable argument. In this examples is used in
 *                  order to select the correct sensor bus handler.
 * @param Reg       register to write
 * @param Bufp      pointer to data to write in register reg
 * @param len       number of consecutive register to write
 * @return int32_t 
 */
static int32_t write_register(void *handle, uint8_t Reg, const uint8_t *Bufp, uint16_t len)
{
    esp_err_t ret = (reinterpret_cast<CPPI2C::I2c *>(handle))->WriteRegisterMultipleBytes(0x6A, Reg, (uint8_t*)Bufp, len);

    if(ret == ESP_OK) {
        return 0;
    }
    else {
        ESP_LOGE(TAG, "Failed to write imu register, error code %d", ret);
        return 1;
    }
}

/**
 * @brief   Read generic device register
 * 
 * @param dev       customizable argument. In this examples is used in
 *                  order to select the correct sensor bus handler.
 * @param Reg       register to read
 * @param Bufp      pointer to buffer that store the data read
 * @param len       number of consecutive register to read
 * @return int32_t 
 */
static int32_t read_register(void *handle, uint8_t Reg, uint8_t *Bufp, uint16_t len)
{
    esp_err_t ret = (reinterpret_cast<CPPI2C::I2c *>(handle))->ReadRegisterMultipleBytes(0x6A, Reg, Bufp, len);

    if(ret == ESP_OK){
        return 0;
    }
    else {
        ESP_LOGE(TAG, "Failed to read imu register, error code %d", ret);
        return 1;
    }
}

LSM6DS3::LSM6DS3(i2c_port_t i2c_port, uint8_t slaveAddress)
{
    slave_address = slaveAddress;

    dev_ctx.write_reg = write_register;
    dev_ctx.read_reg = read_register;
    dev_ctx.handle = &i2c;
}

int LSM6DS3::begin()
{
    static uint8_t whoamI, rst;

    /* Check device ID */
    whoamI = 0;
    lsm6ds3tr_c_device_id_get(&dev_ctx, &whoamI);

    if ( whoamI != LSM6DS3TR_C_ID ) {
        ESP_LOGE(TAG, "IMU not found");
        while (1); /*manage here device not found */
    }
    ESP_LOGI(TAG, "IMU found, 0x%02X", whoamI);
    
    /* Restore default configuration */
    lsm6ds3tr_c_reset_set(&dev_ctx, PROPERTY_ENABLE);

    do {
        lsm6ds3tr_c_reset_get(&dev_ctx, &rst);
    } while (rst);

    /* Enable Block Data Update */
    lsm6ds3tr_c_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate */
    lsm6ds3tr_c_xl_data_rate_set(&dev_ctx, LSM6DS3TR_C_XL_ODR_833Hz);
    lsm6ds3tr_c_gy_data_rate_set(&dev_ctx, LSM6DS3TR_C_GY_ODR_833Hz);
    /* Set full scale */
    lsm6ds3tr_c_xl_full_scale_set(&dev_ctx, LSM6DS3TR_C_2g);
    lsm6ds3tr_c_gy_full_scale_set(&dev_ctx, LSM6DS3TR_C_2000dps);
    
    /* Configure filtering chain(No aux interface) */
    /* Accelerometer - analog filter */
    lsm6ds3tr_c_xl_filter_analog_set(&dev_ctx,
                                    LSM6DS3TR_C_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path ( LPF2 not used )*/
    //lsm6ds3tr_c_xl_lp1_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_LP1_ODR_DIV_4);
    /* Accelerometer - LPF1 + LPF2 path */
    lsm6ds3tr_c_xl_lp2_bandwidth_set(&dev_ctx,
                                    LSM6DS3TR_C_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6ds3tr_c_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6ds3tr_c_xl_hp_bandwidth_set(&dev_ctx, LSM6DS3TR_C_XL_HP_ODR_DIV_100);
    /* Gyroscope - filtering chain */
    lsm6ds3tr_c_gy_band_pass_set(&dev_ctx,
                                LSM6DS3TR_C_HP_260mHz_LP1_STRONG);

    return 0;
}

int LSM6DS3::readAcceleration(float& x, float& y, float& z)
{
    lsm6ds3tr_c_reg_t reg;
    lsm6ds3tr_c_status_reg_get(&dev_ctx, &reg.status_reg);
    static int16_t data_raw_acceleration[3] = {0, 0, 0};

    if (reg.status_reg.xlda) {
        /* Read magnetic field data */
        lsm6ds3tr_c_acceleration_raw_get(&dev_ctx,
                                        data_raw_acceleration);
        x = lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[0]);
        y = lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[1]);
        z = lsm6ds3tr_c_from_fs2g_to_mg(data_raw_acceleration[2]);
    }
    return 0;
}

void LSM6DS3::setDataReadyInt2()
{
    lsm6ds3tr_c_int2_route_t route = {
        .int2_drdy_xl = PROPERTY_ENABLE
    };
    lsm6ds3tr_c_pin_int2_route_set(&dev_ctx, route);
}

void LSM6DS3::end()
{
    
}
