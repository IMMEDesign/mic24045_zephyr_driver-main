#ifndef ZEPHYR_DRIVERS_REGULATOR_MIC24045_H_
#define ZEPHYR_DRIVERS_REGULATOR_MIC24045_H_
// Naming convention above:
// <project>_<folder>_<subfolder>_<filename>_H_
//   Project: ZEPHYR
//   Path: drivers/regulator/mic24045
//   File: mic24045.h
//   Suffix: _H_ to denote a header guard

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>

// MIC24045 register addresses
#define MIC24045_REG_SETTINGS2      0x02
#define MIC24045_REG_VOUT           0x03
#define MIC24045_REG_COMMAND        0x04

// Define step and range — placeholder values, adjust per datasheet
#define MIC24045_MIN_CURRENT_MA     500
#define MIC24045_MAX_CURRENT_MA     5000
#define MIC24045_CURRENT_STEP_MA    500

// MIC24045 commands
#define MIC24045_CIFF               0x01

struct mic24045_config {
    struct i2c_dt_spec i2c;
};

// Voltage conversion helpers
uint8_t mic24045_float_to_vout_reg(float vout);
float mic24045_vout_reg_to_float(uint8_t reg_val);

#endif /* ZEPHYR_DRIVERS_REGULATOR_MIC24045_H_ */
