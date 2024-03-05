/**
 * The following driver code was not written entirely from scratch. Since,
 * ST's datasheet for the vl53l0x does not provide a) any register maps and b)
 * any description of the internal components that are operating to obtain
 * the measurements, the driver had to be reverse engineered from their 
 * API library for the driver. A majority of code in ST's api was bloat 
 * and a lot of book keeping for maintaining meta data about the current
 * configuration of the system. For our cases, we simply needed to initialize
 * the sensor and perform single shot readings. Even in the API code, there
 * are registers being written to with no declared definition and so a lot of
 * the code and register setting patterns are quite obscure. Another benefit
 * of re-writing is we can remove a lot of the non-needed bloat and configure
 * it our I2C interface and error handling interfaces.
 * 
 * The ST API is referenced by the below github where a user has the ST API copied.
 * The majority of functions can be traced  in _api.c.
 * https://github.com/LukeL99/nrf51-vl53l0x-driver/blob/master/core/src/vl53l0x_api.c
 * 
 * It was gathered from what useful information was in the data sheet that there
 * are main functions being called to calibrate:
 *      - data_init()
 *      - static_init()
 *      - perform_ref_calibration()
 * 
 * Other calibrations on init include xtalk, offset, and temperature. For our
 * purposes we can assume the default factory tuned calibrations are "okay". This
 * means we only have to really call the three functions above to get things rolling
 * and start seeing some output.
*/

#include <map>
#include <vector>

#include "driver/vl53l0x.h"


#define RETURN_IF_CONTAINS_ERROR(maybe_error) if (maybe_error.has_value()) { return maybe_error; }
#define RETURN_IF_STATUS_NOT_OK(status, code, message)  \
    if (status != HAL_StatusTypeDef::HAL_OK)    \
    {                                           \
        std::make_optional(std::make_pair(      \
            code,                               \
            message));                          \
    }                                           \

#define GET_BIT(i, arr) ((arr[i / 8] >> (i % 8)) & 0x01)
#define UNSET_BIT(i, arr) (arr[i / 8] &= ~(1 << (i % 8)))

std::map<uint8_t, uint8_t> vcsel_pre_range_phase_check_map = {
    {12, 0x18},
    {14, 0x30},
    {16, 0x40},
    {18, 0x50}
};

std::map<uint8_t, std::vector<uint8_t>> vcsel_final_range_configurations = {
    {8,  {0x10, 0x08, 0x02, 0x0C, 0x01, 0x30, 0x00}},
    {10, {0x28, 0x08, 0x03, 0x09, 0x01, 0x20, 0x00}},
    {12, {0x38, 0x08, 0x03, 0x08, 0x01, 0x20, 0x00}},
    {14, {0x48, 0x08, 0x03, 0x07, 0x01, 0x20, 0x00}}
};


static uint8_t stop_variable;

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

    printf("[VL53L0X]: Performing Reference Calibration\n");
    maybe_error = perform_ref_calibration();
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::Set_Signal_Rate_Limit(float mega_counts_per_second_limit) {
    if (!(mega_counts_per_second_limit >= 0 && mega_counts_per_second_limit < 512)) {
        return std::make_optional(
            std::make_pair(
                ErrorCode::INVALID_MCPS_LIMT,
                "Supported MCPS range is 0-512"
            )
        );
    }

    // Convert to 16 bit floating point, 7 bits used for fractional portion.
    uint16_t limit = mega_counts_per_second_limit * (1 << 7);
    HAL_StatusTypeDef status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, ((uint8_t*)&limit)[1]);
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT + 1, ((uint8_t*)&limit)[0]);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Limit write failure"))

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::Get_Measurement_Timing_Budget(uint32_t& budget) {
    /* All units in micro seconds us */
    const uint32_t start_overhead_us       = 1910;
    const uint32_t end_overhead_us         = 960;
    const uint32_t msrc_overhead_us        = 660;
    const uint32_t tcc_overhead_us         = 590;
    const uint32_t dss_overhead_us         = 690;
    const uint32_t pre_range_overhead_us   = 660;
    const uint32_t final_range_overhead_us = 550;

    // Start and end overheads are always present
    budget += start_overhead_us + end_overhead_us;

    enabled_steps_t enabled_steps;
    timeouts_t timeouts;

    auto maybe_error = get_enabled_sequence_steps(enabled_steps);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    maybe_error = get_sequence_steps_timeouts(enabled_steps, timeouts);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    if (enabled_steps.tcc) {
        budget += timeouts.msrc_dss_tcc_us + tcc_overhead_us;
    }

    if (enabled_steps.dss) {
        budget += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead_us);
    }

    if (enabled_steps.msrc) {
        budget += timeouts.msrc_dss_tcc_us + msrc_overhead_us;
    }

    if (enabled_steps.pre_range) {
        budget += timeouts.pre_range_us + pre_range_overhead_us;
    }

    if (enabled_steps.final_range) {
        budget += timeouts.final_range_us + final_range_overhead_us;
    }

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::Set_Measurement_Timing_Budget(uint32_t budget) {
    /* All units in micro seconds us */
    const uint32_t start_overhead_us       = 1910;
    const uint32_t end_overhead_us         = 960;
    const uint32_t msrc_overhead_us        = 660;
    const uint32_t tcc_overhead_us         = 590;
    const uint32_t dss_overhead_us         = 690;
    const uint32_t pre_range_overhead_us   = 660;
    const uint32_t final_range_overhead_us = 550;

    enabled_steps_t enabled_steps;
    timeouts_t timeouts;

    uint32_t used_budget = start_overhead_us + end_overhead_us;

    auto maybe_error = get_enabled_sequence_steps(enabled_steps);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    maybe_error = get_sequence_steps_timeouts(enabled_steps, timeouts);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    if (enabled_steps.tcc) {
        used_budget += timeouts.msrc_dss_tcc_us + tcc_overhead_us;
    }

    if (enabled_steps.dss) {
        used_budget += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead_us);
    }

    if (enabled_steps.msrc) {
        used_budget += timeouts.msrc_dss_tcc_us + msrc_overhead_us;
    }

    if (enabled_steps.pre_range) {
        used_budget += timeouts.pre_range_us + pre_range_overhead_us;
    }

    if (!enabled_steps.final_range) {
        return {};
    }

    used_budget += final_range_overhead_us;

    if (used_budget > budget) {
        return std::make_optional(
            std::make_pair(
                ErrorCode::INVALID_TIMING_BUDGET,
                std::string("Used budget with timeouts exceeds measurement budget")
            )
        );
    }

    uint32_t new_final_range_timeout_us = budget - used_budget;
    uint16_t new_final_range_mclks = convert_timeout_us_to_mclks(
        new_final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
    if (enabled_steps.pre_range) {
        new_final_range_mclks += timeouts.pre_range_mclks;
    }

    new_final_range_mclks = encode_timeout(new_final_range_mclks);
    HAL_StatusTypeDef status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, ((uint8_t*)&new_final_range_mclks)[1]);
    status = SimpleSlam::I2C_Mem_Write_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, ((uint8_t*)&new_final_range_mclks)[0]);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to final range config timeout register"));

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::Set_Vcsel_Pulse_Period(VcselPulsePeriod period, uint8_t pclks, uint32_t current_measurement_budget) {
    HAL_StatusTypeDef status;
    uint8_t encoded_pclks = (pclks >> 1) - 1;

    enabled_steps_t enabled_steps;
    timeouts_t timeouts;

    get_enabled_sequence_steps(enabled_steps);
    get_sequence_steps_timeouts(enabled_steps, timeouts);

    // Have to adjust PRE_RANGE registers as well as those that depend
    // on the pre-range period.
    if (period == VcselPulsePeriod::PRE_RANGE) {
        auto found_iter = vcsel_pre_range_phase_check_map.find(pclks);
        // Did not find valid pclks configuration
        if (found_iter == vcsel_pre_range_phase_check_map.end()) {
            return std::make_optional(
                std::make_pair(ErrorCode::INVALID_VCSEL_PULSE_PERIOD, 
                std::string("Valid pclks values are 12, 14, 16, and 18")));
        }
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, found_iter->second);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, 
            std::string("Failed to write to PRE_RANGE_CONFIG_VALID_PHASE_HIGH"))

        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
        RETURN_IF_STATUS_NOT_OK(
            status, ErrorCode::I2C_ERROR, std::string("Failed to write to PRE_RANGE_CONFIG_VALID_PHASE_LOW"))

        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VCSEL_PERIOD, encoded_pclks);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, 
            std::string("Failed to write to PRE_RANGE_CONFIG_VCSEL_PERIOD"))

        uint16_t new_pre_range_timeout_mclks =
            convert_timeout_us_to_mclks(timeouts.pre_range_us, pclks);
        
        SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, ((uint8_t*)&new_pre_range_timeout_mclks)[1]);
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, ((uint8_t*)&new_pre_range_timeout_mclks)[0]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to pre-range config timeout register"));

        uint16_t new_msrc_timeout_mclks =
            convert_timeout_us_to_mclks(timeouts.msrc_dss_tcc_us, pclks);

        new_msrc_timeout_mclks = std::min(255, new_msrc_timeout_mclks - 1);

        SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_TIMEOUT_MACROP, new_msrc_timeout_mclks);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to msrc timeout config register"));
    }
    else if (period == VcselPulsePeriod::FINAL_RANGE) {
        auto final_range_item = vcsel_final_range_configurations.find(pclks);
        // Did not find valid pclks configuration
        if (final_range_item == vcsel_final_range_configurations.end()) {
            return std::make_optional(
                std::make_pair(ErrorCode::INVALID_VCSEL_PULSE_PERIOD, 
                std::string("Valid pclks values are 8, 10, 12, and 14")));
        }

        std::vector<uint8_t> configration_values = final_range_item->second;

        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, configration_values[0]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to FINAL_RANGE_CONFIG_VALID_PHASE_HIGH"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, configration_values[1]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to FINAL_RANGE_CONFIG_VALID_PHASE_LOW"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_VCSEL_WIDTH, configration_values[2]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to GLOBAL_CONFIG_VCSEL_WIDTH"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, ALGO_PHASECAL_CONFIG_TIMEOUT, configration_values[3]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to ALGO_PHASECAL_CONFIG_TIMEOUT"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, configration_values[4]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to INTERNAL_TUNING_x2"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, ALGO_PHASECAL_LIM, configration_values[5]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to ALGO_PHASECAL_LIM"));
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, configration_values[6]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to INTERNAL_TUNING_x2"));

        SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VCSEL_PERIOD, pclks);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to FINAL_RANGE_CONFIG_VCSEL_PERIOD"));

        uint16_t new_final_range_mclks = convert_timeout_us_to_mclks(timeouts.final_range_us, pclks);

        if (enabled_steps.pre_range) {
            new_final_range_mclks += timeouts.pre_range_mclks;
        }

        new_final_range_mclks = encode_timeout(new_final_range_mclks);
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, ((uint8_t*)&new_final_range_mclks)[1]);
        status = SimpleSlam::I2C_Mem_Write_Single(
            VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, ((uint8_t*)&new_final_range_mclks)[0]);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not write to final range config timeout register"));
    }

    // Re-Calculate timeouts based with past measurement budget with new pclks.
    Set_Measurement_Timing_Budget(current_measurement_budget);

    perform_ref_calibration();
    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> SimpleSlam::VL53L0X::Perform_Single_Shot_Read(uint16_t& distance) {
    HAL_StatusTypeDef status;

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, stop_variable);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))

    // Set to single shot mode
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, SYSRANGE_MODE_SINGLESHOT);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in Perform_Single_Shot_Read() during measurement setup"))

    uint8_t sysrange_start_val;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, &sysrange_start_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed while waiting for sysrange_start reg val to clear"))
    while (sysrange_start_val & 0x01) {
        status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, &sysrange_start_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed while waiting for sysrange_start reg val to clear"))
    }

    uint8_t result_ready_val;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_INTERRUPT_STATUS, &result_ready_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed while wating for result"))
    while ((result_ready_val & 0x07) == 0) {
        status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_INTERRUPT_STATUS, &result_ready_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed while wating for result"))
    }

    uint16_t buffer = 0;
    // Why 10? Read in hi byte first
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_RANGE_STATUS + 10, (uint8_t*)&buffer);
    buffer = buffer << 8;
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_RANGE_STATUS + 11, (uint8_t*)&buffer);

    // The value will become 8190 if there is no obstacle in its path. So
    // set to 0 as we are not "seeing" anything.
    if (buffer == 8190 || buffer == 8191) {
        buffer = 0;
    }

    distance = buffer;

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_INTERRUPT_CLEAR, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed while wating for result"))
    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::data_init(const VL53L0X_Config_t& config) {
    HAL_StatusTypeDef status;

    uint8_t voltage_setting;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, &voltage_setting);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed performing Voltage setting read"))

    if (config.is_voltage_2v8_mode) {
        voltage_setting |= 0x01;
        status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, voltage_setting);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed performing voltage setting write"))
    }

    // Set I2C Mode to standard
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, I2C_MODE, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed writing I2C Mode"))

    // Performs some sort of internal tuning and gets a stop variable?
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    // This variable is used for some sort of sequencing when doing the measurements?
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, &stop_variable);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init(), reading stop variable?"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() during internal tuning"))

    uint8_t msrc_config_control_val;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_CONTROL, &msrc_config_control_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() while reading msrc config"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_CONTROL, msrc_config_control_val | 0x12);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() while disabling limit checks"))

    // Default Signal rate in mcps
    // VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT is the only one
    // limit we can actually set.
    auto maybe_error = Set_Signal_Rate_Limit(0.25f);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, 0xFF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed in data_init() while setting system sequence config"))

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
    status = SimpleSlam::I2C_Mem_Read(VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ADDR_SIZE_8, reference_spad_bit_array, sizeof(reference_spad_bit_array));
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading reference spads"))

    for (int i = 0; i < 6; i++) {
        printf("[VL53L0X]: Spad %d: %u\n", i, reference_spad_bit_array[i]);
    }

    printf("[VL53L0X]: Setting reference spads from NVM of sensor\n");
    maybe_error = set_reference_spads(
        reference_spad_bit_array, sizeof(reference_spad_bit_array), is_aperature, spad_count);
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    printf("[VL53L0X]: Loading default tuning settings\n");
    maybe_error = load_tuning_settings();
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    printf("[VL53L0X]: Setting GPIO Config\n");
    maybe_error = set_gpio_config();
    RETURN_IF_CONTAINS_ERROR(maybe_error)

    uint32_t measurement_timing_budget = 0;
    Get_Measurement_Timing_Budget(measurement_timing_budget);
    printf("[VL53L0X]: Default Measurement Timing Budget (us) = [%lu]\n", measurement_timing_budget);

    /* Disable MSRC and TCC by default */
    // MSRC =  Minimum Signal Rate Check
    // TCC = Target Centre Check
    uint8_t sequence_config;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading sequence config"))

    // Disabling TCC VL53L0X_SEQUENCESTEP_TCC_MASK = 0xEF
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, sequence_config & 0xEF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed writing sequence config"))

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading sequence config"))

    // Disabling MSRC VL53L0X_SEQUENCESTEP_TCC_MASK = 0xFB
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, sequence_config & 0xFB);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed writing sequence config"))

    Set_Measurement_Timing_Budget(measurement_timing_budget);

    measurement_timing_budget = 0;
    Get_Measurement_Timing_Budget(measurement_timing_budget);
    printf("[VL53L0X]: After setting budget Measurement Timing Budget (us) = [%lu]\n", measurement_timing_budget);

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::reset_device() {
    HAL_StatusTypeDef status;

    // Enable the reset bit by bringing low
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed performing SWR"))

    uint8_t who_am_i_val;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))

    // Wait until device resets who am i value
    while (who_am_i_val != 0) {
        SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))
    }

    HAL_Delay(100);

    // Release the reset bit
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SOFT_RESET_GO2_SOFT_RESET_N, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed performing SWR"))

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed Who am i validation"))

    // Wait until the correct WHO_AM_I code is read
    // Indicates device is booted up.
    while (who_am_i_val == 0x00) {
        SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, VL53L0X_WHO_AM_I, &who_am_i_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed Who am i reading"))
    }

    HAL_Delay(100);

    printf("[VL53L0X]: Who am i?: %x\n", who_am_i_val);
    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::set_reference_spads(uint8_t* reference_spads, uint32_t ref_spad_size, bool is_aperature_spad, uint32_t spad_count) {
    HAL_StatusTypeDef status;

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x1, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads()"))

    
    // is_aperture in vl53l0x_api_calibration.c will become true once
    // once we reach the 12 bit. Indiciating the 12 bit is the start of the 
    // aperature SPADs.

    // Disable any reference spads before then.
    uint16_t aperature_spad_start = is_aperature_spad ? 12 : 0;
    for (uint16_t i = 0; i < aperature_spad_start; i++) {
        UNSET_BIT(i, reference_spads);
    }

    uint16_t number_of_bits = ref_spad_size * 8;
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

    status = SimpleSlam::I2C_Mem_Write(VL53L0X_I2C_DEVICE_ADDRESS, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ADDR_SIZE_8, reference_spads, sizeof(reference_spads));
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed I2C in set_reference_spads() when setting the new enabled spads"))

    return {};
}

// Found in vl53l0x_tuning.h, default tuning values 
std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::load_tuning_settings() {
    HAL_StatusTypeDef status;

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() SYSRANGE_START"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_RANGE_CONFIG, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() SYSTEM_RANGE_CONFIG"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x10, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x10"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x11, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x11"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x24, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x24"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x25, 0xFF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x25"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x75, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x75"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_VALID_PHASE_HIGH"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, ALGO_PHASECAL_LIM, 0x20);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() ALGO_PHASECAL_LIM"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, ALGO_PHASECAL_LIM, 0x09);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() ALGO_PHASECAL_LIM"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x54, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x54"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x31, 0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x31"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x32, 0x03);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x32"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x40, 0x83);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x40"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_TIMEOUT_MACROP, 0x25);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() MSRC_CONFIG_TIMEOUT_MACROP"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_CONTROL, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() MSRC_CONFIG_CONTROL"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_MIN_SNR, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_MIN_SNR"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VCSEL_PERIOD, 0x06);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_VCSEL_PERIOD"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, 0x96);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_VALID_PHASE_LOW"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_VALID_PHASE_HIGH"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_SIGMA_THRESH_HI, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_SIGMA_THRESH_HI"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_SIGMA_THRESH_LO, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_CONFIG_SIGMA_THRESH_LO"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x65, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x65"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x66, 0xA0);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x66"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x22, 0x32);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x22"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x14);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_VALID_PHASE_LOW"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x49, 0xFF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x49"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4A, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x4A"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x7A, 0x0A);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x7A"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x7B, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x7B"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x78, 0x21);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x78"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x23, 0x34);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x23"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x42, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x42"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0xFF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x45, 0x26);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x45"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_TIMEOUT_MACROP, 0x05);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() MSRC_CONFIG_TIMEOUT_MACROP"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x40, 0x40);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x40"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_THRESH_LOW, 0x06);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() SYSTEM_THRESH_LOW"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, CROSSTALK_COMPENSATION_PEAK_RATE_MCPS, 0x1A);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() CROSSTALK_COMPENSATION_PEAK_RATE_MCPS"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x43, 0x40);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x43"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x34, 0x03);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x34"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x35, 0x44);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x35"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x31, 0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x31"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4B, 0x09);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x4B"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4C, 0x05);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x4C"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x4D, 0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x4D"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x45, 0x20);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x45"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_VALID_PHASE_LOW"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_VALID_PHASE_HIGH"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_MIN_SNR, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_MIN_SNR"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VCSEL_PERIOD, 0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_VCSEL_PERIOD"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, 0xFE);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x76, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x76"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x77, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x77"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x0D, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x0D"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() POWER_MANAGEMENT_GO1_POWER_FORCE"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, 0xF8);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() SYSTEM_SEQUENCE_CONFIG"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x8E, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() 0x8E"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() SYSRANGE_START"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() INTERNAL_TUNING_x2"))
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed load_tuning_settings() POWER_MANAGEMENT_GO1_POWER_FORCE"))

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::get_spad_count_and_type(uint32_t& count, bool& is_aperature) {
    // I am guessing that whenever more complicated data needs to be read
    // there is possession of registers/enablements to read the desired 
    // register then we must release all the registers.

    HAL_StatusTypeDef status;
    std::string error_msg("Failed I2C in get_spad_count_and_type()");

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)


    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x06);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)
    
    // No defined register for this reg address
    uint8_t tmp;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, tmp | 0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)


    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x07);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_HISTOGRAM_BIN, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    // Another mystery register
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x94, 0x6B);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x92, &tmp);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    count = tmp & 0x7f;
    is_aperature = (tmp >> 7) & 0x01;

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_HISTOGRAM_BIN, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x06);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, &tmp);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, 0x83, tmp & ~0x04);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, INTERNAL_TUNING_x2, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, POWER_MANAGEMENT_GO1_POWER_FORCE, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, error_msg)

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::set_gpio_config() {
    HAL_StatusTypeDef status;

    /* Set interrupt config to new sample ready */
    // Value associated with VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY in 
    // VL53L0X_SetGPIOConfig();
    const uint8_t VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY = 0x04;
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_INTERRUPT_GPIO_CONFIG, VL53L0X_GPIOFUNCTIONALITY_NEW_MEASURE_READY);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed writing SYSTEM_INTERRUPT_GPIO_CONFIG"))

    uint8_t gpio_hv_mux_active_high_val;
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, GPIO_HV_MUX_ACTIVE_HIGH, &gpio_hv_mux_active_high_val);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading GPIO_HV_MUX_ACTIVE_HIGH"))

    // Makes interrupt pin go active low when data available.
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, GPIO_HV_MUX_ACTIVE_HIGH, gpio_hv_mux_active_high_val & 0xEF);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed updating GPIO_HV_MUX_ACTIVE_HIGH"))

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_INTERRUPT_CLEAR, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed writing SYSTEM_INTERRUPT_CLEAR"))

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::perform_ref_calibration() {
    HAL_StatusTypeDef status;

    uint8_t prev_sequence_config;
    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, &prev_sequence_config);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed perform_ref_calibration()"));

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed perform_ref_calibration()"));
    auto maybe_error = perform_single_ref_calibration(0x40);
    RETURN_IF_CONTAINS_ERROR(maybe_error);

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, 0x02);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed perform_ref_calibration()"));
    maybe_error = perform_single_ref_calibration(0x00);
    RETURN_IF_CONTAINS_ERROR(maybe_error);

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, prev_sequence_config);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed perform_ref_calibration()"));

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::perform_single_ref_calibration(uint8_t vhv_init_byte) {
    HAL_StatusTypeDef status;
    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x01 | vhv_init_byte);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed to write to SYSRANGE_START"));

    uint8_t reg_val;
    SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_INTERRUPT_STATUS, &reg_val);
    while ((reg_val & 0x07) == 0) {
        SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, RESULT_INTERRUPT_STATUS, &reg_val);
        RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed to read from RESULT_INTERRUPT_STATUS"));
    }

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_INTERRUPT_CLEAR, 0x01);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed to write to SYSTEM_INTERRUPT_CLEAR"));

    status = SimpleSlam::I2C_Mem_Write_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSRANGE_START, 0x00);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed to write to SYSRANGE_START"));

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::get_enabled_sequence_steps(enabled_steps_t& steps) {
    // Read which part of the ranging sequence is enabled
    uint8_t sequence_config;
    HAL_StatusTypeDef status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, SYSTEM_SEQUENCE_CONFIG, &sequence_config);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not read sequence config"))
    printf("[VL53L0X]: sequence config %x\n", sequence_config);

    steps.tcc          = (sequence_config >> 4) & 0x1;
    steps.dss          = (sequence_config >> 3) & 0x1;
    steps.msrc         = (sequence_config >> 2) & 0x1;
    steps.pre_range    = (sequence_config >> 6) & 0x1;
    steps.final_range  = (sequence_config >> 7) & 0x1;

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t> 
SimpleSlam::VL53L0X::get_sequence_steps_timeouts(enabled_steps_t& steps, timeouts_t& timeouts) {
    HAL_StatusTypeDef status;
    uint8_t byte_buffer = 0;
    uint16_t two_byte_buffer = 0;
    
    auto maybe_error = get_vcsel_pulse_period(byte_buffer, VcselPulsePeriod::PRE_RANGE);
    RETURN_IF_CONTAINS_ERROR(maybe_error);

    timeouts.pre_range_vcsel_period_pclks = byte_buffer;

    status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, MSRC_CONFIG_TIMEOUT_MACROP, &byte_buffer);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading MSRC config timeout"))

    timeouts.msrc_dss_tcc_mclks = byte_buffer + 1;
    timeouts.msrc_dss_tcc_us = convert_timeout_clocks_to_microseconds(
        timeouts.msrc_dss_tcc_mclks,
        timeouts.pre_range_vcsel_period_pclks
    );

    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, 
        ((uint8_t*)&two_byte_buffer + 1));
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO, 
        (uint8_t*)&two_byte_buffer);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading pre-range config timeout"))

    timeouts.pre_range_mclks = decode_timeout(two_byte_buffer);
    timeouts.pre_range_us = convert_timeout_clocks_to_microseconds(
        timeouts.pre_range_mclks,
        timeouts.pre_range_vcsel_period_pclks
    );
    
    maybe_error = get_vcsel_pulse_period(byte_buffer, VcselPulsePeriod::FINAL_RANGE);
    RETURN_IF_CONTAINS_ERROR(maybe_error);

    timeouts.final_range_vcsel_period_pclks = byte_buffer;

    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, 
       ((uint8_t*)&two_byte_buffer + 1));
    status = SimpleSlam::I2C_Mem_Read_Single(
        VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO, 
       (uint8_t*)&two_byte_buffer);
    RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Failed reading final-range config timeout"))

    timeouts.final_range_mclks = decode_timeout(two_byte_buffer);

    if (steps.pre_range) {
        timeouts.final_range_mclks -= timeouts.pre_range_mclks;
    }

    timeouts.final_range_us = convert_timeout_clocks_to_microseconds(
        timeouts.final_range_mclks,
        timeouts.final_range_vcsel_period_pclks
    );

    return {};
}

std::optional<SimpleSlam::VL53L0X::error_t>  
SimpleSlam::VL53L0X::get_vcsel_pulse_period(uint8_t& pulse_period, VcselPulsePeriod period) {
    HAL_StatusTypeDef status;
    uint8_t read_pulse_period;
    switch (period)
    {
        case SimpleSlam::VL53L0X::VcselPulsePeriod::PRE_RANGE:
            status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, PRE_RANGE_CONFIG_VCSEL_PERIOD, &read_pulse_period);
            RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not read pre-reange vcsel period"))
            pulse_period = (read_pulse_period + 1) << 1;
            break;
        case SimpleSlam::VL53L0X::VcselPulsePeriod::FINAL_RANGE:
            status = SimpleSlam::I2C_Mem_Read_Single(VL53L0X_I2C_DEVICE_ADDRESS, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &read_pulse_period);
            RETURN_IF_STATUS_NOT_OK(status, ErrorCode::I2C_ERROR, std::string("Could not read final-range vcsel period"))
            pulse_period = (read_pulse_period + 1) << 1;
            break;
        default:
            pulse_period = 255;
    } 
    return {};
}

uint32_t 
SimpleSlam::VL53L0X::convert_timeout_clocks_to_microseconds(uint16_t period_mclks, uint16_t period_pclks) {
    uint32_t macro_period_ns = (((2304 * period_pclks) * 1655) + 500) / 1000;
    return ((period_mclks * macro_period_ns) + 500) / 1000;
}

uint16_t 
SimpleSlam::VL53L0X::convert_timeout_us_to_mclks(uint32_t timeout_us, u_int16_t period_pclks) {
    uint32_t macro_period_ns = (((2304 * period_pclks) * 1655) + 500) / 1000;
    return (((timeout_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

uint16_t 
SimpleSlam::VL53L0X::decode_timeout(uint16_t timeout_val) {
    return (uint16_t)((timeout_val & 0x00FF) <<
            (uint16_t)((timeout_val & 0xFF00) >> 8)) + 1;
}

uint16_t 
SimpleSlam::VL53L0X::encode_timeout(uint16_t timeout_mclks) {
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0)
    {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0)
        {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    return 0;
}