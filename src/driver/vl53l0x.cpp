#include "driver/vl53l0x.h"

#define RETURN_IF_CONTAINS_ERROR(maybe_error) if (maybe_error.has_value()) { return maybe_error; }
#define RETURN_IF_STATUS_NOT_OK(status, code, message)  \
    if (status != HAL_StatusTypeDef::HAL_OK)    \
    {                                           \
        std::make_optional(std::make_pair(      \
            code,                               \
            message));                          \
    }                                           \

#define GET_BIT(i, arr) (arr[i / 8] >> (i % 8))
#define UNSET_BIT(i, arr) (arr[i / 8] &= ~(1 << (i % 8)))


std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::Init(const VL53L0X_Config_t& config) {
    printf("[VL53L0X]: Performing Software Reset on Sensor\n");
    auto maybe_error = reset_device();
    RETURN_IF_CONTAINS_ERROR(maybe_error);

    printf("[VL53L0X]: Initializing Data\n");
    maybe_error = data_init(config);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    printf("[VL53L0X]: Performing Static Data Initilization\n");
    maybe_error = static_init(config);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::data_init(const VL53L0X_Config_t& config) {
    HAL_StatusTypeDef status;

    uint8_t voltage_setting;
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &voltage_setting);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed performing Voltage setting read"))

    
    if (config.is_voltage_2v8_mode) {
        voltage_setting |= 0x01;
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 
            voltage_setting);
        RETURN_IF_STATUS_NOT_OK(
            status, ErrorCode::I2C_ERROR, std::string("Failed performing voltage setting write"))
    }

    // Set I2C Mode to standard
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, I2C_MODE, 0x00);
    RETURN_IF_STATUS_NOT_OK(
            status, ErrorCode::I2C_ERROR, std::string("Failed writing I2C Mode"))

    // Performs some sort of internal tuning and gets a stop variable?
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    // This variable is used for some sort of sequencing when doing the measurements?
    uint8_t stop_variable;
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, &stop_variable);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init(), reading stop variable?"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))


    uint8_t msrc_config_control_val;
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_CONTROL, &msrc_config_control_val);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, 
        std::string("Failed in data_init() while reading msrc config"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_CONTROL, msrc_config_control_val | 0x12);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, 
        std::string("Failed in data_init() while disabling limit checks"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, 0xFF);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, 
        std::string("Failed in data_init() while setting system sequence config"))

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::static_init(const VL53L0X_Config_t& config) {
    HAL_StatusTypeDef status;

    uint32_t spad_count = 0;
    bool is_aperature = false;
    auto maybe_error = get_spad_count_and_type(spad_count, is_aperature);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    printf("[VL53L0X]: Spad INFO: %lu, %u\n", spad_count, is_aperature);

    // Read in reference spad settings from NVM, calibrated at factory
    // Because we are not using any sort of glass, the datasheet saids that
    // factory defaults should be okay.
    uint8_t reference_spad_bit_array[6];
    status = SimpleSlam::I2C_Mem_Read(
        VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ADDR_SIZE_8, 
        reference_spad_bit_array, sizeof(reference_spad_bit_array));
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed reading reference spads"))

    for (int i = 0; i < 6; i++) {
        printf("[VL53L0X]: Spad %d: %u\n", i, reference_spad_bit_array[i]);
    }

    printf("[VL53L0X]: Setting reference spads from NVM of sensor\n");
    maybe_error = set_reference_spads(reference_spad_bit_array, is_aperature, spad_count);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    printf("[VL53L0X]: Loading default tuning settings\n");
    maybe_error = load_tuning_settings();
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::reset_device() {
    HAL_StatusTypeDef status;

    // Enable the reset bit by bringing low
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed performing SWR"))

    uint8_t who_am_i_val;
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))

    // Wait until device resets who am i value
    while (who_am_i_val != 0) {
        SimpleSlam::I2C_Mem_Read_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
        RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))
    }

    HAL_Delay(100);

    // Release the reset bit
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed performing SWR"))


    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))

    // Wait until the correct WHO_AM_I code is read
    // Indicates device is booted up.
    while (who_am_i_val == 0x00) {
        SimpleSlam::I2C_Mem_Read_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
        RETURN_IF_STATUS_NOT_OK(
            status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))
    }

    HAL_Delay(100);

    printf("[VL53L0X]: Who am i?: %x\n", who_am_i_val);
    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::set_reference_spads(uint8_t reference_spads[6], bool is_aperature_spad, uint32_t spad_count) {
    HAL_StatusTypeDef status;

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    
    // is_aperture in vl53l0x_api_calibration.c will become true once
    // once we reach the 12 bit. Indiciating the 12 bit is the start of the 
    // aperature SPADs.

    // Disable any reference spads before then.
    uint16_t aperature_spad_start = is_aperature_spad ? 12 : 0;
    for (uint16_t i = 0; i < aperature_spad_start; i++) {
        UNSET_BIT(i, reference_spads);
    }

    uint16_t number_of_bits = sizeof(reference_spads) * 8;
    uint16_t current_index = aperature_spad_start;
    uint16_t enabled_spads = 0;
    while (enabled_spads < spad_count && current_index < number_of_bits) {
        if (GET_BIT(current_index, reference_spads)) {
            enabled_spads++;
        }
        current_index++;
    }

    if (enabled_spads != spad_count) {
        std::make_optional(std::make_pair(     
            ErrorCode::INVALID_REF_SPAD_CONFIG,                              
            std::string("Expected to have found equal amount of enabled spads to spad count")));   
    }

    // Unset remaining SPADs
    while (current_index < number_of_bits) {
        UNSET_BIT(current_index, reference_spads);
        current_index++;
    }

    status = SimpleSlam::I2C_Mem_Write(
        VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ADDR_SIZE_8, 
        reference_spads, sizeof(reference_spads));
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads() when setting the new enabled spads"))

    return {};
}

// Found in vl53l0x_tuning.h, default tuning values 
std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::load_tuning_settings() {

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x00, 0x00);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x09, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x10, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x11, 0x00);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x24, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x25, 0xFF);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x75, 0x00);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4E, 0x2C);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x48, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x30, 0x20);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x30, 0x09);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x54, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x31, 0x04);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x32, 0x03);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x40, 0x83);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x46, 0x25);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x60, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x27, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x50, 0x06);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x51, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x52, 0x96);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x56, 0x08);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x57, 0x30);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x61, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x62, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x64, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x65, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x66, 0xA0);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x22, 0x32);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x47, 0x14);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x49, 0xFF);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4A, 0x00);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x7A, 0x0A);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x7B, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x78, 0x21);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x23, 0x34);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x42, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x44, 0xFF);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x45, 0x26);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x46, 0x05);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x40, 0x40);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x0E, 0x06);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x20, 0x1A);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x43, 0x40);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x34, 0x03);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x35, 0x44);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x31, 0x04);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4B, 0x09);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4C, 0x05);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4D, 0x04);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x44, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x45, 0x20);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x47, 0x08);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x48, 0x28);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x67, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x70, 0x04);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x71, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x72, 0xFE);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x76, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x77, 0x00);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x0D, 0x01);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x80, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x01, 0xF8);

    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x8E, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x00, 0x01);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0xFF, 0x00);
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x80, 0x00);

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::get_spad_count_and_type(uint32_t& count, bool& is_aperature) {
    HAL_StatusTypeDef status;

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))


    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x06);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))
    
    // No defined register for this reg address
    uint8_t tmp;
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, 0x83, tmp | 0x04);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))


    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x07);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_HISTOGRAM_BIN, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    // Another mystery register
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x94, 0x6B);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x92, &tmp);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    count = tmp & 0x7f;
    is_aperature = (tmp >> 7) & 0x01;

    // I am guessing that whenever more complicated data needs to be read
    // there is possession of registers/enablements to read the desired 
    // register then we must release all the registers.
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_HISTOGRAM_BIN, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x06);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, 0x83, tmp & ~0x04);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(
        status, ErrorCode::I2C_ERROR, std::string("Failed I2C in get_spad_count_and_type()"))

    return {};
}


