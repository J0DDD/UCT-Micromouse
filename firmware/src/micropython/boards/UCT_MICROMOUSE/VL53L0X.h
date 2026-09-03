///////////////////////////////////////////////////////////
// Project       : Multi-ToF for UCT MicroMouse
// Microcontroller: STM32L476VE
// Purpose       : To interface multiple VL53L0X ToF sensors
// Made For      : Justin Pead
// Modified By   : Jesse Jabez Arendse
// Original Author: https://github.com/Squieler/VL53L0X---STM32-HAL
// Description   : This file contains the register map, data structures, 
//                 and API functions for the VL53L0X ToF sensor.
// Last Modified : 18-06-2025
///////////////////////////////////////////////////////////

#ifndef VL53L0X_h
#define VL53L0X_h

#include <stdbool.h>

//------------------------------------------------------------
// Register Map and Enums
//------------------------------------------------------------

// System control registers
#define VL53L0X_SYSRANGE_START                          0x00 // 0x00: Single shot mode, 0x01: Continuous mode, 0x02: Back-to-back mode
#define VL53L0X_SYSTEM_THRESH_HIGH                      0x0C // High threshold for range
#define VL53L0X_SYSTEM_THRESH_LOW                       0x0E // Low threshold for range
#define VL53L0X_SYSTEM_SEQUENCE_CONFIG                  0x01 // Sequence configuration
#define VL53L0X_SYSTEM_RANGE_CONFIG                     0x09 // Range configuration
#define VL53L0X_SYSTEM_INTERMEASUREMENT_PERIOD          0x04 // Inter-measurement period
#define VL53L0X_SYSTEM_INTERRUPT_CONFIG_GPIO            0x0A // GPIO interrupt configuration: 0x00 Disabled, 0x01 Low level, 0x02 High level, etc.
#define VL53L0X_GPIO_HV_MUX_ACTIVE_HIGH                 0x84 // GPIO HV MUX active high
#define VL53L0X_SYSTEM_INTERRUPT_CLEAR                  0x0B // Clear system interrupt

/* Interrupt status values */
typedef enum {
    VL53L0X_INTERRUPT_DISABLED      = 0x00, // Disabled
    VL53L0X_INTERRUPT_LEVEL_LOW     = 0x01, // Low level
    VL53L0X_INTERRUPT_LEVEL_HIGH    = 0x02, // High level
    VL53L0X_INTERRUPT_OUT_OF_WINDOW = 0x03, // Out of window
    VL53L0X_INTERRUPT_NEW_SAMPLE_READY = 0x04 // New sample ready
} VL53L0X_InterruptStatus;

// Result registers
#define VL53L0X_RESULT_INTERRUPT_STATUS                 0x13 // Interrupt status
#define VL53L0X_RESULT_RANGE_STATUS                     0x14 // Range status
#define VL53L0X_RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN   0xBC // Core ambient window events (return)
#define VL53L0X_RESULT_CORE_RANGING_TOTAL_EVENTS_RTN    0xC0 // Core ranging total events (return)
#define VL53L0X_RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF   0xD0 // Core ambient window events (reference)
#define VL53L0X_RESULT_CORE_RANGING_TOTAL_EVENTS_REF    0xD4 // Core ranging total events (reference)
#define VL53L0X_RESULT_PEAK_SIGNAL_RATE_REF             0xB6 // Peak signal rate (reference)

// Algorithm registers
#define VL53L0X_ALGO_PART_TO_PART_RANGE_OFFSET_MM       0x28 // Part-to-part range offset in mm

// I2C slave device address
#define VL53L0X_I2C_SLAVE_DEVICE_ADDRESS                0x8A // I2C slave device address
#define VL53L0X_I2C_MODE                                0x88 // I2C mode

typedef enum {
    VL53L0X_I2C_STANDARD_MODE = 0,
    VL53L0X_I2C_FAST_MODE = 2
} VL53L0X_I2C_Mode;

// Check limit registers
#define VL53L0X_MSRC_CONFIG_CONTROL                     0x60 // MSRC configuration control
#define VL53L0X_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44 // Minimum count rate return limit
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_LOW      0x47 // Valid phase low
#define VL53L0X_FINAL_RANGE_CONFIG_VALID_PHASE_HIGH     0x48 // Valid phase high
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI    0x71 // Timeout macro period high

// Pre-range registers
#define VL53L0X_PRE_RANGE_CONFIG_VCSEL_PERIOD           0x50 // VCSEL period for pre-range
#define VL53L0X_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI      0x51 // Timeout macro period high for pre-range
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_HIGH       0x56 // Valid phase high for pre-range
#define VL53L0X_PRE_RANGE_CONFIG_VALID_PHASE_LOW        0x57 // Valid phase low for pre-range

// Final range registers
#define VL53L0X_FINAL_RANGE_CONFIG_VCSEL_PERIOD         0x70 // VCSEL period for final range

// Global configuration registers
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_0        0xB0 // SPAD enables reference 0
#define VL53L0X_GLOBAL_CONFIG_REF_EN_START_SELECT       0xB6 // Reference enable start select
#define VL53L0X_GLOBAL_CONFIG_VCSEL_WIDTH               0x32 // VCSEL width
#define VL53L0X_GLOBAL_CONFIG_SPAD_ENABLES_REF_1        0xB1 // SPAD enables reference 1

// Other configuration registers
#define VL53L0X_VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV       0x89 // VHV configuration for SCL/SDA
#define VL53L0X_ALGO_PHASECAL_LIM                       0x30 // Algorithm phase calibration limit
#define VL53L0X_ALGO_PHASECAL_CONFIG_TIMEOUT            0x30 // Algorithm phase calibration timeout

// Internal tuning registers
#define VL53L0X_INTERNAL_TUNING_1                       0x91 // Internal tuning register 1
#define VL53L0X_INTERNAL_TUNING_2                       0xFF // Internal tuning register 2

// Unknown registers
#define VL53L0X_UNKNOWN_1                               0x85 // Unknown register 1
#define VL53L0X_UNKNOWN_2                               0xCD // Unknown register 2
#define VL53L0X_UNKNOWN_3                               0xCC // Unknown register 3
#define VL53L0X_UNKNOWN_4                               0xBE // Unknown register 4

// Other registers
#define VL53L0X_SYSTEM_HISTOGRAM_BIN                    0x81 // Histogram bin
#define VL53L0X_HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT   0x33 // Histogram initial phase select
#define VL53L0X_HISTOGRAM_CONFIG_READOUT_CTRL           0x55 // Histogram readout control
#define VL53L0X_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO    0x72 // Timeout macro period low for final range
#define VL53L0X_CROSSTALK_COMPENSATION_PEAK_RATE_MCPS   0x20 // Crosstalk compensation peak rate
#define VL53L0X_MSRC_CONFIG_TIMEOUT_MACROP              0x46 // MSRC timeout macro period
#define VL53L0X_SOFT_RESET_GO2_SOFT_RESET_N             0xBF // Soft reset
#define VL53L0X_IDENTIFICATION_MODEL_ID                 0xC0 // Model ID
#define VL53L0X_IDENTIFICATION_REVISION_ID              0xC2 // Revision ID
#define VL53L0X_OSC_CALIBRATE_VAL                       0xF8 // Oscillator calibration value

// Error codes
enum VL53L0X_Error {
    NONE = 0,                          // No error occurred
    VCSELCONTINUITYTESTFAILURE = 1,    // VCSEL continuity test failure
    VCSELWATCHDOGTESTFAILURE = 2,      // VCSEL watchdog test failure
    NOVHVVALUEFOUND = 3,               // No VHV value found
    MSRCNOTARGET = 4,                  // MSRC no target
    SNRCHECK = 5,                      // SNR check failed
    RANGEPHASECHECK = 6,               // Range phase check failed
    SIGMATHRESHOLDCHECK = 7,           // Sigma threshold check failed
    TCC = 8,                           // TCC error
    PHASECONSISTENCY = 9,              // Phase consistency error
    MINCLIP = 10,                      // Minimum clip error
    RANGECOMPLETE = 11,                // Range complete
    ALGOUNDERFLOW = 12,                // Algorithm underflow
    ALGOOVERFLOW = 13,                 // Algorithm overflow
    RANGEIGNORETHRESHOLD = 14          // Range ignore threshold error
};
#define XSHUT1_Pin       GPIO_PIN_8
#define XSHUT1_GPIO_Port GPIOE
#define XSHUT2_Pin       GPIO_PIN_15
#define XSHUT2_GPIO_Port GPIOE
#define XSHUT3_Pin       GPIO_PIN_3
#define XSHUT3_GPIO_Port GPIOE
#define XSHUT4_Pin       GPIO_PIN_10
#define XSHUT4_GPIO_Port GPIOE
#define XSHUT5_Pin       GPIO_PIN_2
#define XSHUT5_GPIO_Port GPIOE
#define XSHUT6_Pin       GPIO_PIN_11
#define XSHUT6_GPIO_Port GPIOC
#define XSHUT7_Pin       GPIO_PIN_3
#define XSHUT7_GPIO_Port GPIOD
#define XSHUT8_Pin       GPIO_PIN_8
#define XSHUT8_GPIO_Port GPIOA
#define XSHUT9_Pin       GPIO_PIN_0
#define XSHUT9_GPIO_Port GPIOE

#define ADDRESS_DEFAULT 0b01010010

// Record the current time to check an upcoming timeout against
#define startTimeout() (g_timeoutStartMs = HAL_GetTick())

// Check if timeout is enabled (set to nonzero value) and has expired
#define checkTimeoutExpired() (g_ioTimeout > 0 && ((uint16_t)HAL_GetTick() - g_timeoutStartMs) > g_ioTimeout)

// Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
// from register value
// based on VL53L0X_decode_vcsel_period()
#define decodeVcselPeriod(reg_val)      (((reg_val) + 1) << 1)

// Encode VCSEL pulse period register value from period in PCLKs
// based on VL53L0X_encode_vcsel_period()
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

// Calculate macro period in *nanoseconds* from VCSEL period in PCLKs
// based on VL53L0X_calc_macro_period_ps()
// PLL_period_ps = 1655; macro_period_vclks = 2304
#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

// register addresses from API vl53l0x_device.h (ordered as listed there)
enum regAddr {
  SYSRANGE_START                              = 0x00,

  SYSTEM_THRESH_HIGH                          = 0x0C,
  SYSTEM_THRESH_LOW                           = 0x0E,

  SYSTEM_SEQUENCE_CONFIG                      = 0x01,
  SYSTEM_RANGE_CONFIG                         = 0x09,
  SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,

  SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,

  GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,

  SYSTEM_INTERRUPT_CLEAR                      = 0x0B,

  RESULT_INTERRUPT_STATUS                     = 0x13,
  RESULT_RANGE_STATUS                         = 0x14,

  RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
  RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
  RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
  RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
  RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,

  ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,

  I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,

  MSRC_CONFIG_CONTROL                         = 0x60,

  PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
  PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
  PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
  PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,

  FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
  FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
  FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
  FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,

  PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
  PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,

  PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
  PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,

  SYSTEM_HISTOGRAM_BIN                        = 0x81,
  HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
  HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,

  FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
  FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
  CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,

  MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,

  SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
  IDENTIFICATION_MODEL_ID                     = 0xC0,
  IDENTIFICATION_REVISION_ID                  = 0xC2,

  OSC_CALIBRATE_VAL                           = 0xF8,

  GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
  GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,

  GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
  DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
  DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
  POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,

  VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,

  ALGO_PHASECAL_LIM                           = 0x30,
  ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
};

typedef enum { VcselPeriodPreRange, VcselPeriodFinalRange }vcselPeriodType;

// Additional info for one measurement
typedef struct {
  uint8_t Address;             /*!< I2C Address */
  I2C_HandleTypeDef *I2Cx;    /*!< I2C peripheral handle */
  GPIO_TypeDef *XSHUT_Port;   /*!< XSHUT GPIO port */
  uint16_t XSHUT_Pin;         /*!< XSHUT GPIO pin */
  uint32_t Distance;           /*!< Distance in millimeters */
  uint32_t Status;             /*!< OK: 0, NOK: !0 */
  uint16_t Ambient;            /*!< Ambient Counting Rate [kcps/spad], fixpoint9.7 */
  uint16_t Signal;             /*!< Signal Counting Rate [kcps/spad], fixpoint9.7 */
  uint16_t rawDistance;        /*!< Uncorrected distance [mm] */
  uint16_t spadCnt;            /*!< Effective SPAD return count, fixpoint8.8 */
  uint8_t rangeStatus;         /*!< Ranging status (0-15) */
  uint32_t timingBudget;
  uint8_t initialized;          /*!< 1 if sensor has been successfully initialized */
} VL53L0_t;

//------------------------------------------------------------
// API Functions
//------------------------------------------------------------
// configures chip i2c and lib for `new_addr` (8 bit, LSB=0)
void setAddress_VL53L0X(uint8_t new_addr);
// Returns the current I²C address.
uint8_t getAddress_VL53L0X(void);

// Iniitializes and configures the sensor. 
// If the optional argument io_2v8 is 1, the sensor is configured for 2V8 mode (2.8 V I/O); 
// if 0, the sensor is left in 1V8 mode. Returns 1 if the initialization completed successfully.
bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler);

// Sets the return signal rate limit to the given value in units of MCPS (mega counts per second). 
// This is the minimum amplitude of the signal reflected from the target and received by the sensor 
//  necessary for it to report a valid reading. Setting a lower limit increases the potential range 
// of the sensor but also increases the likelihood of getting an inaccurate reading because of 
//  reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS 
//  by default. The return value is a boolean indicating whether the requested limit was valid.
bool setSignalRateLimit(uint16_t limit_kcps);

// Returns the current return signal rate limit in MCPS.
float getSignalRateLimit(void);

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(uint32_t budget_us);

// Returns the current measurement timing budget in microseconds.
uint32_t getMeasurementTimingBudget(void);

// Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type
// (VcselPeriodPreRange or VcselPeriodFinalRange) to the given value (in PCLKs). 
// Longer periods increase the potential range of the sensor. Valid values are (even numbers only):
// Pre: 12 to 18 (initialized to 14 by default)
// Final: 8 to 14 (initialized to 10 by default)
// The return value is a boolean indicating whether the requested period was valid.
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

// Returns the current VCSEL pulse period for the given period type.
uint8_t getVcselPulsePeriod(vcselPeriodType type);

// Starts continuous ranging measurements. If the argument period_ms is 0, 
// continuous back-to-back mode is used (the sensor takes measurements as often as possible); 
// if it is nonzero, continuous timed mode is used, with the specified inter-measurement period 
// in milliseconds determining how often the sensor takes a measurement.
void startContinuous(uint32_t period_ms);

// Stops continuous mode.
void stopContinuous(void);

// Returns a range reading in millimeters when continuous mode is active.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeContinuousMillimeters(VL53L0_t *extraStats);

// Performs a single-shot ranging measurement and returns the reading in millimeters.
// Additional measurement data will be copied into `extraStats` if it is non-zero.
uint16_t readRangeSingleMillimeters(VL53L0_t *extraStats);

// Sets a timeout period in milliseconds after which read operations will abort 
// if the sensor is not ready. A value of 0 disables the timeout.
void setTimeout(uint16_t timeout);

// Returns the current timeout period setting.
uint16_t getTimeout(void);

// Indicates whether a read timeout has occurred since the last call to timeoutOccurred().
bool timeoutOccurred(void);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
void writeReg(uint8_t reg, uint8_t value);        // Write an 8-bit register
void writeReg16Bit(uint8_t reg, uint16_t value);  // Write a 16-bit register
void writeReg32Bit(uint8_t reg, uint32_t value);  // Write a 32-bit register
uint8_t readReg(uint8_t reg);                     // Read an 8-bit register
uint16_t readReg16Bit(uint8_t reg);               // Read a 16-bit register
uint32_t readReg32Bit(uint8_t reg);               // Read a 32-bit register
// Write `count` number of bytes from `src` to the sensor, starting at `reg`
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count);
// Read `count` number of bytes from the sensor, starting at `reg`, to `dst`
void readMulti(uint8_t reg, uint8_t *dst, uint8_t count);


void initTOFs(uint16_t signalRate);
void refreshTOFValues();

// TCC: Target CentreCheck
// MSRC: Minimum Signal Rate Check
// DSS: Dynamic Spad Selection
typedef struct {
  uint8_t tcc, msrc, dss, pre_range, final_range;
}SequenceStepEnables;

typedef struct {
  uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

  uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
  uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
}SequenceStepTimeouts;

// VL53L0_t sensor instances (defined in VL53L0X.c)
extern VL53L0_t TOF_sb_left_result;
extern VL53L0_t TOF_sb_front_result;
extern VL53L0_t TOF_sb_right_result;
extern VL53L0_t TOF_sb_front_left_result;
extern VL53L0_t TOF_sb_front_right_result;
extern VL53L0_t TOF_mb_back_result;
extern VL53L0_t TOF_mb_front_result;
extern VL53L0_t TOF_mb_front_left_result;
extern VL53L0_t TOF_mb_front_right_result;

#endif
