#include "driver/i2c.h"

static I2C_HandleTypeDef i2c_handler;

static void scl_sda_gpio_init() {
    SCL_SDA_GPIO_CLK_ENABLE();

    GPIO_InitTypeDef scl_sda_init_data;
    scl_sda_init_data.Pin = SCL_PIN | SDA_PIN;
    scl_sda_init_data.Mode = GPIO_MODE_AF_OD;
    scl_sda_init_data.Pull = GPIO_PULLUP;
    scl_sda_init_data.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    scl_sda_init_data.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(SCL_SDA_GPIO_PORT, &scl_sda_init_data);
}

static void scl_sda_gpio_deinit() {
    GPIO_InitTypeDef scl_sda_init_data;
    scl_sda_init_data.Pin = SCL_PIN | SDA_PIN;

    HAL_GPIO_DeInit(SCL_SDA_GPIO_PORT, scl_sda_init_data.Pin);
    SCL_SDA_GPIO_CLK_DISABLE();
    INTERNAL_I2C_CLK_DISABLE();
}

static void handle_i2c_error() {
    SimpleSlam::I2C_DeInit();
    SimpleSlam::I2C_Init();
}

void SimpleSlam::I2C_Init() {
    // Setup the I2C initialization parameters
    i2c_handler.Instance              = I2C2;
    i2c_handler.Init.Timing           = I2C_TIMINGR;
    i2c_handler.Init.OwnAddress1      = 0;
    i2c_handler.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
    i2c_handler.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
    i2c_handler.Init.OwnAddress2      = 0;
    i2c_handler.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    i2c_handler.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
    i2c_handler.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;

    scl_sda_gpio_init();

    INTERNAL_I2C_CLK_ENABLE();

    // Perform an I2C software reset
    // reference: RM0351 Section 39.4.6: Software reset, see Figure 392.
    INTERNAL_I2C_FORCE_RESET();
    INTERNAL_I2C_RELEASE_RESET();

    HAL_I2CEx_ConfigAnalogFilter(&i2c_handler, I2C_ANALOGFILTER_ENABLE); 

    HAL_I2C_Init(&i2c_handler);
}

void SimpleSlam::I2C_DeInit() {
    scl_sda_gpio_deinit();
    HAL_I2C_DeInit(&i2c_handler);
}

HAL_StatusTypeDef SimpleSlam::I2C_Mem_Write(
    uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, 
    uint8_t *buffer, uint16_t size
) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &i2c_handler, peripheral_address, reg_address, 
        reg_address_size, buffer, size, TIMEOUT_US);

    if (status != HAL_OK) {
       handle_i2c_error();
    }
    return status;
}

HAL_StatusTypeDef SimpleSlam::I2C_Mem_Write_Single(
    uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, 
    uint8_t *value
) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
        &i2c_handler, peripheral_address, reg_address,  reg_address_size, 
        value, SINGLE_SIZE, TIMEOUT_US
    );

    if (status != HAL_OK) {
       handle_i2c_error();
    }
    return status;
}

HAL_StatusTypeDef SimpleSlam::I2C_Mem_Read(
    uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, 
    uint8_t *buffer, uint16_t size
) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &i2c_handler, peripheral_address, reg_address, reg_address_size, 
        buffer, size, TIMEOUT_US
    );

    if (status != HAL_OK) {
        handle_i2c_error();
    }
    return status;
}


HAL_StatusTypeDef SimpleSlam::I2C_Mem_Read_Single(
    uint16_t peripheral_address, uint16_t reg_address, uint16_t reg_address_size, 
    uint8_t *buffer
) {
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
        &i2c_handler, peripheral_address, reg_address, reg_address_size, buffer, 
        SINGLE_SIZE, TIMEOUT_US
    );

    if (status != HAL_OK) {
        handle_i2c_error();
    }
    return status;
}
