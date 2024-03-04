/**
 * VL53L0X Module Driver
 * Resources:
*/

#pragma once

#include <string>
#include <optional>
#include "driver/i2c.h"

namespace SimpleSlam::VL53L0X {
/** 
 * ST does not provide a register map in any of their data sheets for VL53L0X
 * Have to use crowd sourced register Map: 
 *   - https://github.com/GrimbiXcode/VL53L0X-Register-Map
*/

/**
 *  https://www.st.com/resource/en/datasheet/vl53l0x.pdf
*/ 
#define VL53L0X_I2C_DEVICE_ADDRESS              0x52
#define VL53L0X_WHO_AM_I	                    0xC0
#define VL53L0X_EXPECTED_WHO_AM_I_VALUE	        0xEE

#define SYSRANGE_START	            0x00
// SYSRANGE_START[0] 
#define SYSRANGE_MODE_START_STOP    0x0
// SYSRANGE_START[1] 
#define SYSRANGE_MODE_SINGLESHOT    (0x0 << 1)
#define SYSRANGE_MODE_BACK_TO_BACK  (0x1 << 1)
// SYSRANGE_START[2] 
#define SYSRANGE_MODE_TIMED         (0x1 << 2)
// SYSRANGE_START[3] 
#define SYSRANGE_MODE_HISTOGRAM     (0x1 << 3)

#define SYSTEM_THRESH_HIGH	            0x0C	
#define SYSTEM_THRESH_LOW	            0x0E	
#define SYSTEM_SEQUENCE_CONFIG	        0x01	
#define SYSTEM_RANGE_CONFIG	            0x09	
#define SYSTEM_INTERMEASUREMENT_PERIOD	0x04	

/* GPIO config */
#define SYSTEM_INTERRUPT_GPIO_CONFIG	0x0A
#define GPIO_HV_MUX_ACTIVE_HIGH	        0x84	
#define SYSTEM_INTERRUPT_CLEAR	        0x0B	
#define I2C_SLAVE_DEVICE_ADDRESS	    0x8a	
#define I2C_MODE	                    0x88

/* Result registers */
#define RESULT_INTERRUPT_STATUS	                0x13	
#define RESULT_RANGE_STATUS	                    0x14	
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN	0xBC	
#define RESULT_CORE_RANGING_TOTAL_EVENTS_RTN	0xC0	
#define RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF	0xD0	
#define RESULT_CORE_RANGING_TOTAL_EVENTS_REF	0xD4	
#define RESULT_PEAK_SIGNAL_RATE_REF	            0xB6	

/* Algo register */
#define ALGO_PART_TO_PART_RANGE_OFFSET_MM	    0x28	

/* Check Limit registers */
#define MSRC_CONFIG_CONTROL	                0x60	

#define PRE_RANGE_CONFIG_MIN_SNR	        0x27	
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW	0x56	
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH	0x57	
#define PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT	0x64	

#define FINAL_RANGE_CONFIG_MIN_SNR	                0x67	
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW	        0x47	
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH	        0x48	
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT	0x44	

#define PRE_RANGE_CONFIG_SIGMA_THRESH_HI	0x61	
#define PRE_RANGE_CONFIG_SIGMA_THRESH_LO	0x62	
#define PRE_RANGE_CONFIG_VCSEL_PERIOD	    0x50	
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI	0x51	
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO	0x52	

#define SYSTEM_HISTOGRAM_BIN	                0x81	
#define HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT	0x33	
#define HISTOGRAM_CONFIG_READOUT_CTRL	        0x55	
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD	        0x70	
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI	0x71	
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO	0x72	
#define CROSSTALK_COMPENSATION_PEAK_RATE_MCPS	0x20	


/**  
 * Mystery Registers? Do not really not what these are doing.
 * However they seemed to be activated in the ST API source code?
*/
#define INTERNAL_TUNING_x1	0x91	
#define INTERNAL_TUNING_x2	0xFF    // I believe this register activates some sort of meta state when writing/reading data from other registers.

/* Other Registers */
#define MSRC_CONFIG_TIMEOUT_MACROP	        0x46	
#define SOFT_RESET_GO2_SOFT_RESET_N	        0xBF	
#define IDENTIFICATION_MODEL_ID	            0xC0	
#define IDENTIFICATION_REVISION_ID	        0xC2	
#define OSC_CALIBRATE_VAL	                0xF8	
#define GLOBAL_CONFIG_VCSEL_WIDTH	        0x032	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0	0x0B0	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_1	0x0B1	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_2	0x0B2	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_3	0x0B3	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_4	0x0B4	
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_5	0x0B5	
#define GLOBAL_CONFIG_REF_EN_START_SELECT	0xB6	
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD	0x4E	
#define DYNAMIC_SPAD_REF_EN_START_OFFSET	0x4F	
#define POWER_MANAGEMENT_GO1_POWER_FORCE	0x80    // 0x00 is Power Mode Idle and 0x01 is power mode standby. Seems then when making configuration changes, neeed to bring power mode into idle.	
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV	0x89	
#define ALGO_PHASECAL_LIM	                0x30	
#define ALGO_PHASECAL_CONFIG_TIMEOUT	    0x30

enum class ErrorCode {
    I2C_ERROR = 1,
    WHO_AM_I_UNEXPECTED_VALUE = 2,
    INVALID_REF_SPAD_CONFIG = 3,
};

typedef std::pair<ErrorCode, std::string> error_t;

typedef struct VL53L0X_Config {
    bool is_voltage_2v8_mode;
} VL53L0X_Config_t;

std::optional<error_t> Init(const VL53L0X_Config_t& config);

std::optional<error_t> data_init(const VL53L0X_Config_t& config);
std::optional<error_t> static_init(const VL53L0X_Config_t& config);
std::optional<error_t> reset_device();
std::optional<error_t> set_reference_spads(uint8_t* reference_spads, uint32_t ref_spad_size, bool is_aperature_spad, uint32_t spad_count);
std::optional<error_t> load_tuning_settings();
std::optional<error_t> get_spad_count_and_type(uint32_t& count, bool& is_aperature);

}