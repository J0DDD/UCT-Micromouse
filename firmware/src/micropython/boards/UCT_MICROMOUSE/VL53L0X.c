///////////////////////////////////////////////////////////
// Project : Multi-ToF for UCT MicroMouse
// Microcontroller: STM32L476VE
// Made For : Justin Pead
// @modified: Jesse Jabez Arendse
// @author: https://github.com/Squieler/VL53L0X---STM32-HAL
///////////////////////////////////////////////////////////


// Most of the functionality of this library is based on the VL53L0X API
// provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
// or paraphrased from the API source code, API user manual (UM2039), and the
// VL53L0X datasheet.

#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h" // Change it for your requirements.
#include <stdio.h>
#include "string.h"
#include "VL53L0X.h"
#include "main.h"

#ifdef I2C_TIMEOUT
#undef I2C_TIMEOUT
#endif

__attribute__((weak)) void raw_uart_print(const char *str) {
    (void)str;
}
#define I2C_TIMEOUT 2 // I2C timeout in ms
#define I2C_READ 1
#define I2C_WRITE 0

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

uint32_t TOF_Distance[9];
uint16_t TOF_Ambient[9];
uint16_t TOF_Signal[9];
uint32_t TOF_Status[9];


//---------------------------------------------------------
// Local variables within this file (private)
//---------------------------------------------------------
uint8_t g_i2cAddr = ADDRESS_DEFAULT;
uint16_t g_ioTimeout = 0;  // no timeout
uint8_t g_isTimeout = 0;
uint16_t g_timeoutStartMs;
uint8_t g_stopVariable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
uint32_t g_measTimBudUs;


uint8_t VL53L0_address_default = 0x52;
uint8_t newToFAddress_L  = 0x54;
uint8_t newToFAddress_FL = 0x5A;
uint8_t newToFAddress_C  = 0x60;
uint8_t newToFAddress_FR = 0x66;
uint8_t newToFAddress_R  = 0x6C;
uint8_t newToFAddress_B   = 0x72;
uint8_t newToFAddress_MB_F  = 0x78;
uint8_t newToFAddress_MB_FL = 0x7E;
uint8_t newToFAddress_MB_FR = 0x84;
VL53L0_t TOF_sb_left_result;
VL53L0_t TOF_sb_front_result;
VL53L0_t TOF_sb_right_result;
VL53L0_t TOF_sb_front_left_result;
VL53L0_t TOF_sb_front_right_result;
VL53L0_t TOF_mb_back_result;
VL53L0_t TOF_mb_front_result;
VL53L0_t TOF_mb_front_left_result;
VL53L0_t TOF_mb_front_right_result;

I2C_HandleTypeDef *VL53L0X_I2C_Handler; // I2C handler
uint8_t msgBuffer[4];
HAL_StatusTypeDef i2cStat;

//---------------------------------------------------------
// Locally used functions (private)
//---------------------------------------------------------
bool getSpadInfo(uint8_t *count, bool *type_is_aperture);
void getSequenceStepEnables(SequenceStepEnables * enables);
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);
bool performSingleRefCalibration(uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

//---------------------------------------------------------
// I2C communication Functions
//---------------------------------------------------------
// Write an 8-bit register
void writeReg(uint8_t reg, uint8_t value) {

  msgBuffer[0] = value; // Assign the value to the buffer.
  i2cStat = HAL_I2C_Mem_Write(VL53L0X_I2C_Handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
}

// Write a 16-bit register
void writeReg16Bit(uint8_t reg, uint16_t value){

  memcpy(msgBuffer, &value, 2); // Assign the value to the buffer.
  i2cStat = HAL_I2C_Mem_Write(VL53L0X_I2C_Handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
}

// Write a 32-bit register
void writeReg32Bit(uint8_t reg, uint32_t value){

  memcpy(msgBuffer, &value, 4); // Assign the value to the buffer.
  i2cStat = HAL_I2C_Mem_Write(VL53L0X_I2C_Handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
}

// Read an 8-bit register
uint8_t readReg(uint8_t reg) {
  uint8_t value;

  i2cStat = HAL_I2C_Mem_Read(VL53L0X_I2C_Handler, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 1, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
  value = msgBuffer[0];

  return value;
}

// Read a 16-bit register
uint16_t readReg16Bit(uint8_t reg) {
  uint16_t value;

  i2cStat = HAL_I2C_Mem_Read(VL53L0X_I2C_Handler, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 2, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
  memcpy(&value, msgBuffer, 2);

  return value;
}

// Read a 32-bit register
uint32_t readReg32Bit(uint8_t reg) {
  uint32_t value;
  i2cStat = HAL_I2C_Mem_Read(VL53L0X_I2C_Handler, g_i2cAddr | I2C_READ, reg, 1, msgBuffer, 4, I2C_TIMEOUT);
  if (i2cStat != HAL_OK) {
    restartI2C(VL53L0X_I2C_Handler);
  }
  memcpy(&value, msgBuffer, 4);

  return value;
}

// Write an arbitrary number of bytes from the given array to the sensor,
// starting at the given register
void writeMulti(uint8_t reg, uint8_t const *src, uint8_t count){

  memcpy(msgBuffer, src, 4);
  i2cStat = HAL_I2C_Mem_Write(VL53L0X_I2C_Handler, g_i2cAddr | I2C_WRITE, reg, 1, msgBuffer, count, I2C_TIMEOUT);
}

// Read an arbitrary number of bytes from the sensor, starting at the given
// register, into the given array
void readMulti(uint8_t reg, uint8_t * dst, uint8_t count) {

	i2cStat = HAL_I2C_Mem_Read(VL53L0X_I2C_Handler, g_i2cAddr | I2C_READ, reg, 1, dst, count, I2C_TIMEOUT);
}


// Public Methods //////////////////////////////////////////////////////////////

void setAddress_VL53L0X(uint8_t new_addr) {
  writeReg( I2C_SLAVE_DEVICE_ADDRESS, (new_addr>>1) & 0x7F );
}

uint8_t getAddress_VL53L0X() {
  return g_i2cAddr;
}

// Initialize sensor using sequence based on VL53L0X_DataInit(),
// VL53L0X_StaticInit(), and VL53L0X_PerformRefCalibration().
// This function does not perform reference SPAD calibration
// (VL53L0X_PerformRefSpadManagement()), since the API user manual says that it
// is performed by ST on the bare modules; it seems like that should work well
// enough unless a cover glass is added.
// If io_2v8 (optional) is true or not given, the sensor is configured for 2V8
// mode.
bool initVL53L0X(bool io_2v8, I2C_HandleTypeDef *handler){
  // VL53L0X_DataInit() begin

  // Handler
  VL53L0X_I2C_Handler = handler;

  // Reset the message buffer.
  msgBuffer[0] = 0;
  msgBuffer[1] = 0;
  msgBuffer[2] = 0;
  msgBuffer[3] = 0;

  // sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
  if (io_2v8)
  {
    writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
      readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); // set bit 0
  }

  // "Set I2C standard mode"
  writeReg(VL53L0X_I2C_MODE, VL53L0X_I2C_FAST_MODE);

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  g_stopVariable = readReg(0x91);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks
  writeReg(MSRC_CONFIG_CONTROL, readReg(MSRC_CONFIG_CONTROL) | 0x12);

  // set final range signal rate limit to 0.25 MCPS (million counts per second)
  setSignalRateLimit(200);

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);

  // VL53L0X_DataInit() end

  // VL53L0X_StaticInit() begin

  uint8_t spad_count;
  bool spad_type_is_aperture;
  if (!getSpadInfo(&spad_count, &spad_type_is_aperture)) { return false; }

  // The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
  // the API, but the same data seems to be more easily readable from
  // GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there
  uint8_t ref_spad_map[6];
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() begin (assume NVM values are valid)

  writeReg(0xFF, 0x01);
  writeReg(DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
  writeReg(DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
  writeReg(0xFF, 0x00);
  writeReg(GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

  uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0; // 12 is the first aperture spad
  uint8_t spads_enabled = 0;

  for (uint8_t i = 0; i < 48; i++)
  {
    if (i < first_spad_to_enable || spads_enabled == spad_count)
    {
      // This bit is lower than the first one that should be enabled, or
      // (reference_spad_count) bits have already been enabled, so zero this bit
      ref_spad_map[i / 8] &= ~(1 << (i % 8));
    }
    else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
    {
      spads_enabled++;
    }
  }

  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

  // -- VL53L0X_set_reference_spads() end

  // -- VL53L0X_load_tuning_settings() begin
  // DefaultTuningSettings from vl53l0x_tuning.h

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x09, 0x00);
  writeReg(0x10, 0x00);
  writeReg(0x11, 0x00);

  writeReg(0x24, 0x01);
  writeReg(0x25, 0xFF);
  writeReg(0x75, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x4E, 0x2C);
  writeReg(0x48, 0x00);
  writeReg(0x30, 0x20);

  writeReg(0xFF, 0x00);
  writeReg(0x30, 0x09);
  writeReg(0x54, 0x00);
  writeReg(0x31, 0x04);
  writeReg(0x32, 0x03);
  writeReg(0x40, 0x83);
  writeReg(0x46, 0x25);
  writeReg(0x60, 0x00);
  writeReg(0x27, 0x00);
  writeReg(0x50, 0x06);
  writeReg(0x51, 0x00);
  writeReg(0x52, 0x96);
  writeReg(0x56, 0x08);
  writeReg(0x57, 0x30);
  writeReg(0x61, 0x00);
  writeReg(0x62, 0x00);
  writeReg(0x64, 0x00);
  writeReg(0x65, 0x00);
  writeReg(0x66, 0xA0);

  writeReg(0xFF, 0x01);
  writeReg(0x22, 0x32);
  writeReg(0x47, 0x14);
  writeReg(0x49, 0xFF);
  writeReg(0x4A, 0x00);

  writeReg(0xFF, 0x00);
  writeReg(0x7A, 0x0A);
  writeReg(0x7B, 0x00);
  writeReg(0x78, 0x21);

  writeReg(0xFF, 0x01);
  writeReg(0x23, 0x34);
  writeReg(0x42, 0x00);
  writeReg(0x44, 0xFF);
  writeReg(0x45, 0x26);
  writeReg(0x46, 0x05);
  writeReg(0x40, 0x40);
  writeReg(0x0E, 0x06);
  writeReg(0x20, 0x1A);
  writeReg(0x43, 0x40);

  writeReg(0xFF, 0x00);
  writeReg(0x34, 0x03);
  writeReg(0x35, 0x44);

  writeReg(0xFF, 0x01);
  writeReg(0x31, 0x04);
  writeReg(0x4B, 0x09);
  writeReg(0x4C, 0x05);
  writeReg(0x4D, 0x04);

  writeReg(0xFF, 0x00);
  writeReg(0x44, 0x00);
  writeReg(0x45, 0x20);
  writeReg(0x47, 0x08);
  writeReg(0x48, 0x28);
  writeReg(0x67, 0x00);
  writeReg(0x70, 0x04);
  writeReg(0x71, 0x01);
  writeReg(0x72, 0xFE);
  writeReg(0x76, 0x00);
  writeReg(0x77, 0x00);

  writeReg(0xFF, 0x01);
  writeReg(0x0D, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x01);
  writeReg(0x01, 0xF8);

  writeReg(0xFF, 0x01);
  writeReg(0x8E, 0x01);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  // -- VL53L0X_load_tuning_settings() end

  // "Set interrupt config to new sample ready"
  // -- VL53L0X_SetGpioConfig() begin

  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, VL53L0X_INTERRUPT_NEW_SAMPLE_READY);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  // -- VL53L0X_SetGpioConfig() end

  g_measTimBudUs = getMeasurementTimingBudget();

  // "Disable MSRC and TCC by default"
  // MSRC = Minimum Signal Rate Check
  // TCC = Target CentreCheck
  // -- VL53L0X_SetSequenceStepEnable() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // -- VL53L0X_SetSequenceStepEnable() end

  // "Recalculate timing budget"
  setMeasurementTimingBudget(g_measTimBudUs);

  // VL53L0X_StaticInit() end

  // VL53L0X_PerformRefCalibration() begin (VL53L0X_perform_ref_calibration())

  // -- VL53L0X_perform_vhv_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return false; }

  // -- VL53L0X_perform_vhv_calibration() end

  // -- VL53L0X_perform_phase_calibration() begin

  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return false; }

  // -- VL53L0X_perform_phase_calibration() end

  // "restore the previous Sequence Config"
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xE8);

  // VL53L0X_PerformRefCalibration() end

  return true;
}

bool setSignalRateLimit(uint16_t limit_kcps) {
    // Convert kcps (kilo counts per second) to Q9.7 fixed-point format
    if (limit_kcps > 51199) { // Maximum value in Q9.7 is 511.99 MCPS
        return false;
    }
    uint16_t limit_q9_7 = (limit_kcps << 7) / 1000; // Convert to Q9.7 format
    writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_q9_7);
    return true;
}

void initVL53L0(VL53L0_t *tof, uint16_t signalRate){
  HAL_GPIO_WritePin(tof->XSHUT_Port, tof->XSHUT_Pin, GPIO_PIN_SET);
  HAL_Delay(50);

  // Set the global I2C handler for this sensor
  VL53L0X_I2C_Handler = tof->I2Cx;
  g_i2cAddr = ADDRESS_DEFAULT;

  uint8_t status = 0;
  status = initVL53L0X(1, tof->I2Cx);
  if (status == false){ 
    restartI2C(tof->I2Cx);
    return;
  }

  uint8_t PreRange = 18;
  uint8_t FinalRange = 14;

	// Configure the sensor for high accuracy and speed in 20 cm.
	if (!setSignalRateLimit(signalRate)) { restartI2C(tof->I2Cx); } // Updated to use uint16_t signalRate
  if (!setVcselPulsePeriod(VcselPeriodPreRange, PreRange)) { restartI2C(tof->I2Cx); }
  if (!setVcselPulsePeriod(VcselPeriodFinalRange, FinalRange)) { restartI2C(tof->I2Cx); }
  // Set timing budget to 100ms (10Hz) to ensure reliable measurements under the long-range profile (PreRange=18, FinalRange=14), falling back to 200ms (5Hz) if needed.
  if (!setMeasurementTimingBudget(100000)) {
    if (!setMeasurementTimingBudget(200000)) {
      restartI2C(tof->I2Cx);
    }
  }
  startContinuous(0);
  setAddress_VL53L0X(tof->Address);
  tof->initialized = 1;

}

void calibrateToF(VL53L0_t* TOF_result , uint16_t distance) {
    uint8_t PreRange = 12; // Default PreRange
    uint8_t FinalRange = 8; // Default FinalRange
    uint32_t timingBudget = 33000; // Default timing budget in microseconds (33 ms)

    // Adjust PreRange, FinalRange, and timing budget based on the signal rate
    if (distance > 500 ) { // Signal rate < 0.12 Mcps
        PreRange = 18; // Max range
        FinalRange = 14; // Max range
        timingBudget = 100000; // 66 ms
    } else { // Signal rate >= 0.40 Mcps
        PreRange = 12; // Shorter range for faster updates
        FinalRange = 8; // Shorter range for faster updates
        timingBudget = 100000; // 10 ms
    }

    // Apply new configurations
    if (TOF_result->timingBudget != timingBudget) {
      TOF_result->timingBudget = timingBudget;
      stopContinuous();
      setMeasurementTimingBudget(timingBudget);
      if (!setVcselPulsePeriod(VcselPeriodPreRange, PreRange)) { restartI2C(TOF_result->I2Cx); }
      if (!setVcselPulsePeriod(VcselPeriodFinalRange, FinalRange)) { restartI2C(TOF_result->I2Cx); }
      startContinuous(0);
    }
}


void getVL53L0(VL53L0_t* TOF_result){
  if (!TOF_result->initialized) { return; }

  // Avoid spamming I2C timeout reads if a sensor is failing/disconnected (error back-off)
  static uint32_t last_fail_time[9] = {0};
  int idx = -1;
  extern VL53L0_t TOF_sb_left_result, TOF_sb_front_left_result, TOF_sb_front_result, TOF_sb_front_right_result, TOF_sb_right_result;
  extern VL53L0_t TOF_mb_back_result, TOF_mb_front_result, TOF_mb_front_left_result, TOF_mb_front_right_result;
  if (TOF_result == &TOF_sb_left_result) idx = 0;
  else if (TOF_result == &TOF_sb_front_left_result) idx = 1;
  else if (TOF_result == &TOF_sb_front_result) idx = 2;
  else if (TOF_result == &TOF_sb_front_right_result) idx = 3;
  else if (TOF_result == &TOF_sb_right_result) idx = 4;
  else if (TOF_result == &TOF_mb_back_result) idx = 5;
  else if (TOF_result == &TOF_mb_front_result) idx = 6;
  else if (TOF_result == &TOF_mb_front_left_result) idx = 7;
  else if (TOF_result == &TOF_mb_front_right_result) idx = 8;

  uint32_t now = HAL_GetTick();
  if (idx >= 0 && last_fail_time[idx] > 0 && (now - last_fail_time[idx] < 100)) {
      return; // Cool-down active: skip read to save tick execution budget
  }

  VL53L0X_I2C_Handler = TOF_result->I2Cx;
  g_i2cAddr = TOF_result->Address;

  VL53L0_t distanceStr; // Updated type
  distanceStr.rangeStatus = 255;
  uint16_t distance = readRangeContinuousMillimeters(&distanceStr);

  if (i2cStat != HAL_OK) {
    TOF_result->Distance = 8190;
    TOF_result->Status = 255;
    if (idx >= 0) {
        last_fail_time[idx] = now;
    }
    return;
  }

  // If a timeout occurred (no new sample ready yet), keep the last cached value
  if (distance == 65535) {
    return;
  }

  // If the range is valid, update the TOF_result structure
  if (distanceStr.rangeStatus == RANGECOMPLETE || distanceStr.rangeStatus == NONE) {
    if (distance < 8000){
      TOF_result->Distance = distance;
      TOF_result->Status = distanceStr.rangeStatus;
      TOF_result->Ambient = distanceStr.Ambient;
      TOF_result->Signal = distanceStr.Signal;
    } else {
      TOF_result->Distance = 8190;
      TOF_result->Status = distanceStr.rangeStatus;
    }
  } else {
    // Ranging failed/timed out (e.g. out of range / open air)
    TOF_result->Distance = 8190;
    TOF_result->Status = distanceStr.rangeStatus;
  }

  
  #ifdef TOF_DYNAMIC_FSR
    calibrateToF(TOF_result , distance);
  #endif
}

void refreshTOFValues() {
    getVL53L0(&TOF_sb_left_result);
    getVL53L0(&TOF_sb_front_left_result);
    getVL53L0(&TOF_sb_front_result);
    getVL53L0(&TOF_sb_front_right_result);
    getVL53L0(&TOF_sb_right_result);
    getVL53L0(&TOF_mb_back_result);
    getVL53L0(&TOF_mb_front_result);
    getVL53L0(&TOF_mb_front_left_result);
    getVL53L0(&TOF_mb_front_right_result);

    TOF_Distance[0] = TOF_sb_left_result.Distance;
    TOF_Distance[1] = TOF_sb_front_left_result.Distance;
    TOF_Distance[2] = TOF_sb_front_result.Distance;
    TOF_Distance[3] = TOF_sb_front_right_result.Distance;
    TOF_Distance[4] = TOF_sb_right_result.Distance;
    TOF_Distance[5] = TOF_mb_back_result.Distance;
    TOF_Distance[6] = TOF_mb_front_result.Distance;
    TOF_Distance[7] = TOF_mb_front_left_result.Distance;
    TOF_Distance[8] = TOF_mb_front_right_result.Distance;

    TOF_Ambient[0] = TOF_sb_left_result.Ambient;
    TOF_Ambient[1] = TOF_sb_front_left_result.Ambient;
    TOF_Ambient[2] = TOF_sb_front_result.Ambient;
    TOF_Ambient[3] = TOF_sb_front_right_result.Ambient;
    TOF_Ambient[4] = TOF_sb_right_result.Ambient;
    TOF_Ambient[5] = TOF_mb_back_result.Ambient;
    TOF_Ambient[6] = TOF_mb_front_result.Ambient;
    TOF_Ambient[7] = TOF_mb_front_left_result.Ambient;
    TOF_Ambient[8] = TOF_mb_front_right_result.Ambient;

    TOF_Signal[0] = TOF_sb_left_result.Signal;
    TOF_Signal[1] = TOF_sb_front_left_result.Signal;
    TOF_Signal[2] = TOF_sb_front_result.Signal;
    TOF_Signal[3] = TOF_sb_front_right_result.Signal;
    TOF_Signal[4] = TOF_sb_right_result.Signal;
    TOF_Signal[5] = TOF_mb_back_result.Signal;
    TOF_Signal[6] = TOF_mb_front_result.Signal;
    TOF_Signal[7] = TOF_mb_front_left_result.Signal;
    TOF_Signal[8] = TOF_mb_front_right_result.Signal;

    TOF_Status[0] = TOF_sb_left_result.Status;
    TOF_Status[1] = TOF_sb_front_left_result.Status;
    TOF_Status[2] = TOF_sb_front_result.Status;
    TOF_Status[3] = TOF_sb_front_right_result.Status;
    TOF_Status[4] = TOF_sb_right_result.Status;
    TOF_Status[5] = TOF_mb_back_result.Status;
    TOF_Status[6] = TOF_mb_front_result.Status;
    TOF_Status[7] = TOF_mb_front_left_result.Status;
    TOF_Status[8] = TOF_mb_front_right_result.Status;
}

void Early_XSHUT_Init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

  GPIO_InitStruct.Pin = XSHUT1_Pin;
  HAL_GPIO_Init(XSHUT1_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT2_Pin;
  HAL_GPIO_Init(XSHUT2_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT3_Pin;
  HAL_GPIO_Init(XSHUT3_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT3_GPIO_Port, XSHUT3_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT4_Pin;
  HAL_GPIO_Init(XSHUT4_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT4_GPIO_Port, XSHUT4_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT5_Pin;
  HAL_GPIO_Init(XSHUT5_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT5_GPIO_Port, XSHUT5_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT6_Pin;
  HAL_GPIO_Init(XSHUT6_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT6_GPIO_Port, XSHUT6_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT7_Pin;
  HAL_GPIO_Init(XSHUT7_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT7_GPIO_Port, XSHUT7_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT8_Pin;
  HAL_GPIO_Init(XSHUT8_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT8_GPIO_Port, XSHUT8_Pin, GPIO_PIN_SET);

  GPIO_InitStruct.Pin = XSHUT9_Pin;
  HAL_GPIO_Init(XSHUT9_GPIO_Port, &GPIO_InitStruct);
  HAL_GPIO_WritePin(XSHUT9_GPIO_Port, XSHUT9_Pin, GPIO_PIN_SET);
}

void initTOFs(uint16_t signalRate){
  extern void raw_uart_print(const char *str);
  raw_uart_print("initTOFs entered!\r\n");
  char buf[64];
  uint32_t primask = __get_PRIMASK();
  snprintf(buf, sizeof(buf), "  PRIMASK interrupt status: %u\r\n", (unsigned int)primask);
  raw_uart_print(buf);

  TOF_sb_left_result.Address = newToFAddress_L;         TOF_sb_left_result.I2Cx = &hi2c2;        TOF_sb_left_result.XSHUT_Port = XSHUT1_GPIO_Port; TOF_sb_left_result.XSHUT_Pin = XSHUT1_Pin; TOF_sb_left_result.Distance = 8190;
  TOF_sb_front_left_result.Address = newToFAddress_FL;  TOF_sb_front_left_result.I2Cx = &hi2c2;  TOF_sb_front_left_result.XSHUT_Port = XSHUT4_GPIO_Port; TOF_sb_front_left_result.XSHUT_Pin = XSHUT4_Pin; TOF_sb_front_left_result.Distance = 8190;
  TOF_sb_front_result.Address = newToFAddress_C;       TOF_sb_front_result.I2Cx = &hi2c2;       TOF_sb_front_result.XSHUT_Port = XSHUT2_GPIO_Port; TOF_sb_front_result.XSHUT_Pin = XSHUT2_Pin; TOF_sb_front_result.Distance = 8190;
  TOF_sb_front_right_result.Address = newToFAddress_FR; TOF_sb_front_right_result.I2Cx = &hi2c2;  TOF_sb_front_right_result.XSHUT_Port = XSHUT5_GPIO_Port; TOF_sb_front_right_result.XSHUT_Pin = XSHUT5_Pin; TOF_sb_front_right_result.Distance = 8190;
  TOF_sb_right_result.Address = newToFAddress_R;        TOF_sb_right_result.I2Cx = &hi2c2;        TOF_sb_right_result.XSHUT_Port = XSHUT3_GPIO_Port; TOF_sb_right_result.XSHUT_Pin = XSHUT3_Pin; TOF_sb_right_result.Distance = 8190;
  TOF_mb_back_result.Address = newToFAddress_B;         TOF_mb_back_result.I2Cx = &hi2c1;         TOF_mb_back_result.XSHUT_Port = XSHUT6_GPIO_Port; TOF_mb_back_result.XSHUT_Pin = XSHUT6_Pin; TOF_mb_back_result.Distance = 8190;
  TOF_mb_front_result.Address = newToFAddress_MB_F;      TOF_mb_front_result.I2Cx = &hi2c2;         TOF_mb_front_result.XSHUT_Port = XSHUT7_GPIO_Port; TOF_mb_front_result.XSHUT_Pin = XSHUT7_Pin; TOF_mb_front_result.Distance = 8190;
  TOF_mb_front_left_result.Address = newToFAddress_MB_FL; TOF_mb_front_left_result.I2Cx = &hi2c2;   TOF_mb_front_left_result.XSHUT_Port = XSHUT8_GPIO_Port; TOF_mb_front_left_result.XSHUT_Pin = XSHUT8_Pin; TOF_mb_front_left_result.Distance = 8190;
  TOF_mb_front_right_result.Address = newToFAddress_MB_FR; TOF_mb_front_right_result.I2Cx = &hi2c2; TOF_mb_front_right_result.XSHUT_Port = XSHUT9_GPIO_Port; TOF_mb_front_right_result.XSHUT_Pin = XSHUT9_Pin; TOF_mb_front_right_result.Distance = 8190;

  setTimeout(200);

  // Reset all sensors
  HAL_GPIO_WritePin(XSHUT1_GPIO_Port, XSHUT1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT2_GPIO_Port, XSHUT2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT3_GPIO_Port, XSHUT3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT4_GPIO_Port, XSHUT4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT5_GPIO_Port, XSHUT5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT6_GPIO_Port, XSHUT6_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT7_GPIO_Port, XSHUT7_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT8_GPIO_Port, XSHUT8_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(XSHUT9_GPIO_Port, XSHUT9_Pin, GPIO_PIN_RESET);
  HAL_Delay(200);

  // Enable each sensor one at a time and initialise it
  extern void raw_uart_print(const char *str);
  raw_uart_print("  initTOFs: initializing left...\r\n");
  initVL53L0(&TOF_sb_left_result, signalRate);
  raw_uart_print("  initTOFs: left done.\r\n");

  raw_uart_print("  initTOFs: initializing front-left...\r\n");
  initVL53L0(&TOF_sb_front_left_result, signalRate);
  raw_uart_print("  initTOFs: front-left done.\r\n");

  raw_uart_print("  initTOFs: initializing front...\r\n");
  initVL53L0(&TOF_sb_front_result, signalRate);
  raw_uart_print("  initTOFs: front done.\r\n");

  raw_uart_print("  initTOFs: initializing front-right...\r\n");
  initVL53L0(&TOF_sb_front_right_result, signalRate);
  raw_uart_print("  initTOFs: front-right done.\r\n");

  raw_uart_print("  initTOFs: initializing right...\r\n");
  initVL53L0(&TOF_sb_right_result, signalRate);
  raw_uart_print("  initTOFs: right done.\r\n");

  raw_uart_print("  initTOFs: initializing back...\r\n");
  initVL53L0(&TOF_mb_back_result, signalRate);
  raw_uart_print("  initTOFs: back done.\r\n");

  raw_uart_print("  initTOFs: initializing main-front...\r\n");
  initVL53L0(&TOF_mb_front_result, signalRate);
  raw_uart_print("  initTOFs: main-front done.\r\n");

  raw_uart_print("  initTOFs: initializing main-front-left...\r\n");
  initVL53L0(&TOF_mb_front_left_result, signalRate);
  raw_uart_print("  initTOFs: main-front-left done.\r\n");

  raw_uart_print("  initTOFs: initializing main-front-right...\r\n");
  initVL53L0(&TOF_mb_front_right_result, signalRate);
  raw_uart_print("  initTOFs: main-front-right done.\r\n");

  setTimeout(5);
}

// Set the return signal rate limit check value in units of MCPS (mega counts
// per second). "This represents the amplitude of the signal reflected from the
// target and detected by the device"; setting this limit presumably determines
// the minimum measurement necessary for the sensor to report a valid reading.
// Setting a lower limit increases the potential range of the sensor but also
// seems to increase the likelihood of getting an inaccurate reading because of
// unwanted reflections from objects other than the intended target.
// Defaults to 0.25 MCPS as initialized by the ST API and this library.
// bool setSignalRateLimit(uint16_t limit_Mcps)
// {
//   if (limit_Mcps < 0 || limit_Mcps > 511.99) { return false; }

//   // Q9.7 fixed point format (9 integer bits, 7 fractional bits)
//   writeReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, limit_Mcps * (1 << 7));
//   return true;
// }

// Get the return signal rate limit check value in MCPS
float getSignalRateLimit(void)
{
  return (float)readReg16Bit(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT) / (1 << 7);
}

// Set the measurement timing budget in microseconds, which is the time allowed
// for one measurement; the ST API and this library take care of splitting the
// timing budget among the sub-steps in the ranging sequence. A longer timing
// budget allows for more accurate measurements. Increasing the budget by a
// factor of N decreases the range measurement standard deviation by a factor of
// sqrt(N). Defaults to about 33 milliseconds; the minimum is 20 ms.
// based on VL53L0X_set_measurement_timing_budget_micro_seconds()
bool setMeasurementTimingBudget(uint32_t budget_us)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  uint32_t const MinTimingBudget = 20000;

  if (budget_us < MinTimingBudget) { return false; }

  uint32_t used_budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    used_budget_us += FinalRangeOverhead;

    // "Note that the final range timeout is determined by the timing
    // budget and the sum of all other timeouts within the sequence.
    // If there is no room for the final range timeout, then an error
    // will be set. Otherwise the remaining time will be applied to
    // the final range."

    if (used_budget_us > budget_us)
    {
      // "Requested timeout too big."
      return false;
    }

    uint32_t final_range_timeout_us = budget_us - used_budget_us;

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(final_range_timeout_us,
                                 timeouts.final_range_vcsel_period_pclks);

    if (enables.pre_range)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(final_range_timeout_mclks));

    // set_sequence_step_timeout() end

    g_measTimBudUs = budget_us; // store for internal reuse
  }
  return true;
}

// Get the measurement timing budget in microseconds
// based on VL53L0X_get_measurement_timing_budget_micro_seconds()
// in us
uint32_t getMeasurementTimingBudget(void)
{
  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;

  // "Start and end overhead times always present"
  uint32_t budget_us = StartOverhead + EndOverhead;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  if (enables.tcc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }

  if (enables.dss)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables.msrc)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }

  if (enables.pre_range)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }

  if (enables.final_range)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }

  g_measTimBudUs = budget_us; // store for internal reuse
  return budget_us;
}

// Set the VCSEL (vertical cavity surface emitting laser) pulse period for the
// given period type (pre-range or final range) to the given value in PCLKs.
// Longer periods seem to increase the potential range of the sensor.
// Valid values are (even numbers only):
//  pre:  12 to 18 (initialized default: 14)
//  final: 8 to 14 (initialized default: 10)
// based on VL53L0X_set_vcsel_pulse_period()
bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);

  SequenceStepEnables enables;
  SequenceStepTimeouts timeouts;

  getSequenceStepEnables(&enables);
  getSequenceStepTimeouts(&enables, &timeouts);

  // "Apply specific settings for the requested clock period"
  // "Re-calculate and apply timeouts, in macro periods"

  // "When the VCSEL period for the pre or final range is changed,
  // the corresponding timeout must be read from the device using
  // the current VCSEL period, then the new VCSEL period can be
  // applied. The timeout then must be written back to the device
  // using the new VCSEL period.
  //
  // For the MSRC timeout, the same applies - this timeout being
  // dependant on the pre-range vcsel period."


  if (type == VcselPeriodPreRange)
  {
    // "Set phase check limits"
    switch (period_pclks)
    {
      case 12:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;

      case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;

      case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;

      case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;

      default:
        // invalid period
        return false;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);

    // apply new VCSEL period
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_PRE_RANGE)

    uint16_t new_pre_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);

    writeReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_pre_range_timeout_mclks));

    // set_sequence_step_timeout() end

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_MSRC)

    uint16_t new_msrc_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);

    writeReg(MSRC_CONFIG_TIMEOUT_MACROP,
      (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));

    // set_sequence_step_timeout() end
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;

      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      default:
        // invalid period
        return false;
    }

    // apply new VCSEL period
    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);

    // update timeouts

    // set_sequence_step_timeout() begin
    // (SequenceStepId == VL53L0X_SEQUENCESTEP_FINAL_RANGE)

    // "For the final range timeout, the pre-range timeout
    //  must be added. To do this both final and pre-range
    //  timeouts must be expressed in macro periods MClks
    //  because they have different vcsel periods."

    uint16_t new_final_range_timeout_mclks =
      timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);

    if (enables.pre_range)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }

    writeReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
      encodeTimeout(new_final_range_timeout_mclks));

    // set_sequence_step_timeout end
  }
  else
  {
    // invalid type
    return false;
  }

  // "Finally, the timing budget must be re-applied"

  setMeasurementTimingBudget(g_measTimBudUs);

  // "Perform the phase calibration. This is needed after changing on vcsel period."
  // VL53L0X_perform_phase_calibration() begin

  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);

  // VL53L0X_perform_phase_calibration() end

  return true;
}

// Get the VCSEL pulse period in PCLKs for the given period type.
// based on VL53L0X_get_vcsel_pulse_period()
uint8_t getVcselPulsePeriod(vcselPeriodType type)
{
  if (type == VcselPeriodPreRange)
  {
    return decodeVcselPeriod(readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else if (type == VcselPeriodFinalRange)
  {
    return decodeVcselPeriod(readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD));
  }
  else { return 255; }
}

// Start continuous ranging measurements. If period_ms (optional) is 0 or not
// given, continuous back-to-back mode is used (the sensor takes measurements as
// often as possible); otherwise, continuous timed mode is used, with the given
// inter-measurement period in milliseconds determining how often the sensor
// takes a measurement.
// based on VL53L0X_StartMeasurement()
void startContinuous(uint32_t period_ms)
{
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, g_stopVariable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  if (period_ms != 0)
  {
    // continuous timed mode

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() begin

    uint16_t osc_calibrate_val = readReg16Bit(OSC_CALIBRATE_VAL);

    if (osc_calibrate_val != 0)
    {
      period_ms *= osc_calibrate_val;
    }

    writeReg32Bit(SYSTEM_INTERMEASUREMENT_PERIOD, period_ms);

    // VL53L0X_SetInterMeasurementPeriodMilliSeconds() end

    writeReg(SYSRANGE_START, 0x04); // VL53L0X_REG_SYSRANGE_MODE_TIMED
  }
  else
  {
    // continuous back-to-back mode
    writeReg(SYSRANGE_START, 0x02); // VL53L0X_REG_SYSRANGE_MODE_BACKTOBACK
  }
}

// Stop continuous measurements
// based on VL53L0X_StopMeasurement()
void stopContinuous(void)
{
  writeReg(SYSRANGE_START, 0x01); // VL53L0X_REG_SYSRANGE_MODE_SINGLESHOT

  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, 0x00);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
}

// Returns a range reading in millimeters when continuous mode is active
// (readRangeSingleMillimeters() also calls this function after starting a
// single-shot range measurement)
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t readRangeContinuousMillimeters( VL53L0_t *extraStats) {
  uint8_t tempBuffer[12];
  uint16_t temp;
  #ifdef COMPILED_BY_SIMULINK
  if ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (extraStats != NULL) {
      extraStats->rangeStatus = 255; // Custom status indicating "not ready"
    }
    return 65535;
  }
  #else
  startTimeout();
  volatile uint32_t timeout_counter = 10000;
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
    if (--timeout_counter == 0 || i2cStat != HAL_OK || checkTimeoutExpired())
    {
      g_isTimeout = true;
      return 65535;
    }
  }
  #endif
  if (extraStats == NULL) {
    // assumptions: Linearity Corrective Gain is 1000 (default);
    // fractional ranging is not enabled
    temp = readReg16Bit(RESULT_RANGE_STATUS + 10);
  } else {
    // Register map starting at 0x14
    //     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F
    //    5A 06 BC 04 00 85 00 38 00 19 06 B6 00 00 00 00
    //   0: Ranging status, uint8_t
    //   1: ???
    // 3,2: Effective SPAD return count, uint16_t, fixpoint8.8
    //   4: 0 ?
    //   5: ???
    // 6,7: signal count rate [mcps], uint16_t, fixpoint9.7
    // 9,8: AmbientRateRtnMegaCps  [mcps], uint16_t, fixpoimt9.7
    // A,B: uncorrected distance [mm], uint16_t
    readMulti(0x14, tempBuffer, 12);
    extraStats->rangeStatus =  tempBuffer[0x00]>>3;
    extraStats->spadCnt     = (tempBuffer[0x02]<<8) | tempBuffer[0x03];
    extraStats->Signal   = (tempBuffer[0x06]<<8) | tempBuffer[0x07];
    extraStats->Ambient  = (tempBuffer[0x08]<<8) | tempBuffer[0x09];    
    temp                    = (tempBuffer[0x0A]<<8) | tempBuffer[0x0B];
    extraStats->rawDistance = temp;
  }
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return temp;
}

// Performs a single-shot range measurement and returns the reading in
// millimeters
// based on VL53L0X_PerformSingleRangingMeasurement()
// extraStats provides additional info for this measurment. Set to 0 if not needed.
uint16_t readRangeSingleMillimeters( VL53L0_t *extraStats ) {
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, g_stopVariable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(SYSRANGE_START, 0x01);
  // "Wait until start bit has been cleared"
  startTimeout();
  volatile uint32_t timeout_counter = 10000;
  while (readReg(SYSRANGE_START) & 0x01) {
    if (--timeout_counter == 0 || i2cStat != HAL_OK || checkTimeoutExpired()) {
      g_isTimeout = true;
      return 65535;
    }
  }
  return readRangeContinuousMillimeters(extraStats);
}

// Did a timeout occur in one of the read functions since the last call to
// timeoutOccurred()?
bool timeoutOccurred()
{
  bool tmp = g_isTimeout;
  g_isTimeout = false;
  return tmp;
}

void setTimeout(uint16_t timeout){
  g_ioTimeout = timeout;
}

uint16_t getTimeout(void){
  return g_ioTimeout;
}

// Private Methods /////////////////////////////////////////////////////////////

// Get reference SPAD (single photon avalanche diode) count and type
// based on VL53L0X_get_info_from_device(),
// but only gets reference SPAD count and type
bool getSpadInfo(uint8_t * count, bool * type_is_aperture)
{
  uint8_t tmp;

  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);

  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeReg(0xFF, 0x07);
  writeReg(0x81, 0x01);

  writeReg(0x80, 0x01);

  writeReg(0x94, 0x6b);
  writeReg(0x83, 0x00);
  startTimeout();
  volatile uint32_t timeout_counter = 10000;
  while (readReg(0x83) == 0x00)
  {
    if (--timeout_counter == 0 || i2cStat != HAL_OK || checkTimeoutExpired()) { return false; }
  }
  writeReg(0x83, 0x01);
  tmp = readReg(0x92);

  *count = tmp & 0x7f;
  *type_is_aperture = (tmp >> 7) & 0x01;

  writeReg(0x81, 0x00);
  writeReg(0xFF, 0x06);
  writeReg(0x83, readReg(0x83)  & ~0x04);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x01);

  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);

  return true;
}

// Get sequence step enables
// based on VL53L0X_GetSequenceStepEnables()
void getSequenceStepEnables(SequenceStepEnables * enables)
{
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);

  enables->tcc          = (sequence_config >> 4) & 0x1;
  enables->dss          = (sequence_config >> 3) & 0x1;
  enables->msrc         = (sequence_config >> 2) & 0x1;
  enables->pre_range    = (sequence_config >> 6) & 0x1;
  enables->final_range  = (sequence_config >> 7) & 0x1;
}

// Get sequence step timeouts
// based on get_sequence_step_timeout(),
// but gets all timeouts instead of just the requested one, and also stores
// intermediate values
void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodPreRange);

  timeouts->msrc_dss_tcc_mclks = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us =
    timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->pre_range_mclks =
    decodeTimeout(readReg16Bit(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us =
    timeoutMclksToMicroseconds(timeouts->pre_range_mclks,
                               timeouts->pre_range_vcsel_period_pclks);

  timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(VcselPeriodFinalRange);

  timeouts->final_range_mclks =
    decodeTimeout(readReg16Bit(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

  if (enables->pre_range)
  {
    timeouts->final_range_mclks -= timeouts->pre_range_mclks;
  }

  timeouts->final_range_us =
    timeoutMclksToMicroseconds(timeouts->final_range_mclks,
                               timeouts->final_range_vcsel_period_pclks);
}

// Decode sequence step timeout in MCLKs from register value
// based on VL53L0X_decode_timeout()
// Note: the original function returned a uint32_t, but the return value is
// always stored in a uint16_t.
uint16_t decodeTimeout(uint16_t reg_val)
{
  // format: "(LSByte * 2^MSByte) + 1"
  return (uint16_t)((reg_val & 0x00FF) <<
         (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
// based on VL53L0X_encode_timeout()
// Note: the original function took a uint16_t, but the argument passed to it
// is always a uint16_t.
uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  // format: "(LSByte * 2^MSByte) + 1"

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
  else { return 0; }
}

// Convert sequence step timeout from MCLKs to microseconds with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_us()
uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

// Convert sequence step timeout from microseconds to MCLKs with given VCSEL period in PCLKs
// based on VL53L0X_calc_timeout_mclks()
uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);

  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}


// based on VL53L0X_perform_single_ref_calibration()
bool performSingleRefCalibration(uint8_t vhv_init_byte)
{
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); // VL53L0X_REG_SYSRANGE_MODE_START_STOP

  startTimeout();
  volatile uint32_t timeout_counter = 10000;
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    if (--timeout_counter == 0 || i2cStat != HAL_OK || checkTimeoutExpired()) { return false; }
  }

  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);

  writeReg(SYSRANGE_START, 0x00);

  return true;
}
