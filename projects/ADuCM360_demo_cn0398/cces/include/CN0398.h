#ifndef _CN0398_H_
#define _CN0398_H_
#include "AD7124.h"

#define RTD_CHANNEL        0
#define PH_CHANNEL         1
#define MOISTURE_CHANNEL   2


#define R0        100.0

#define YES       1
#define NO        0

/**
 * @brief Calibration solutions enum
 */
enum {
    ACETATE,
    BORATE,
    CAOH2,
    CARBONATE,
    CITRATE,
    HCL,
    OXALATE,
    PHOSPHATE0,
    PHOSPHATE1,
    PHOSPHATE2,
    PHTHALATE,
    TARTRATE,
    TRIS,
    PH4,
    PH10,
    NUMBER_OF_SOLUTIONS
};

/**
 * @brief Calibration solutions strings
 */
const char solutions[NUMBER_OF_SOLUTIONS][20] =
{
      "ACETATE",
       "BORATE",
       "CAOH2",
       "CARBONATE",
       "CITRATE",
       "HCL",
       "OXALATE",
       "PHOSPHATE0",
       "PHOSPHATE1",
       "PHOSPHATE2",
       "PHTHALATE",
       "TARTRATE",
       "TRIS",
       "PH4",
       "PH10"
};

#define NUMBER_OF_TEMPERATURE_ENTRIES 31

// *INDENT-OFF*
const uint8_t ph_temperatures[NUMBER_OF_TEMPERATURE_ENTRIES] =
{
		0 ,		5 ,		10,		15,		18,		19,		20,		21,		22,		23,		24,		25,
		26,		27,		28,		29,		30,		35,		37,		40,		45,		50,		55,		60,
		65,		70,		75,		80,		85,		90,		95,
};

/**
 * @brief Calibration solutions temperature to ph look-up tables
 */
const float ph_temp_lut[NUMBER_OF_SOLUTIONS][NUMBER_OF_TEMPERATURE_ENTRIES]
{
/* ACETATE    */ {4.667, 4.66, 4.655, 4.652, 4.651, 4.651, 4.65, 4.65, 4.65, 4.65, 4.65, 4.65, 4.65, 4.651, 4.651, 4.651, 4.652, 4.655, 4.656, 4.659, 4.666, 4.673, 4.683, 4.694, 4.706, 4.72, 4.736, 4.753, 4.772, 4.793, 4.815},
/* BORATE     */ {9.464, 9.395, 9.332, 9.276, 9.245, 9.235, 9.225, 9.216, 9.207, 9.197, 9.189, 9.18, 9.171, 9.163, 9.155, 9.147, 9.139, 9.102, 9.088, 9.068, 9.038, 9.01, 8.985, 8.962, 8.941, 8.921, 8.902, 8.884, 8.867, 8.85, 8.833},
/* CAOH2      */ {13.424, 13.207, 13.003, 12.81, 12.699, 12.663, 12.627, 12.592, 12.557, 12.522, 12.488, 12.454, 12.42, 12.387, 12.354, 12.322, 12.289, 12.133, 12.072, 11.984, 11.841, 11.705, 11.574, 11.449 },
/* CARBONATE  */ {10.317, 10.245, 10.179, 10.118, 10.084, 10.073, 10.062, 10.052, 10.042, 10.032, 10.022, 10.012, 10.002, 9.993, 9.984, 9.975, 9.966, 9.925, 9.91, 9.889, 9.857, 9.828},
/* CITRATE    */ {3.863, 3.84, 3.82, 3.803, 3.793, 3.791, 3.788, 3.785, 3.783, 3.78, 3.778, 3.776, 3.774, 3.772, 3.77, 3.768, 3.766, 3.759, 3.756, 3.754, 3.75, 3.749},
/* HCL        */ {1.082, 1.085, 1.087, 1.089, 1.09, 1.091, 1.091, 1.092, 1.092, 1.093, 1.093, 1.094, 1.094, 1.094, 1.095, 1.095, 1.096, 1.098, 1.099, 1.101, 1.103, 1.106, 1.108, 1.111, 1.113, 1.116, 1.119, 1.121, 1.124, 1.127, 1.13},
/* OXALATE    */ {1.666, 1.668, 1.67, 1.672, 1.674, 1.675, 1.675, 1.676, 1.677, 1.678, 1.678, 1.679, 1.68, 1.681, 1.681, 1.682, 1.683, 1.688, 1.69, 1.694, 1.7, 1.707, 1.715, 1.723, 1.732, 1.743, 1.754, 1.765, 1.778, 1.792, 1.806},
/* PHOSPHATE0 */ {6.984, 6.951, 6.923, 6.9, 6.888, 6.884, 6.881, 6.877, 6.874, 6.871, 6.868, 6.865, 6.862, 6.86, 6.857, 6.855, 6.853, 6.844, 6.841, 6.838, 6.834, 6.833, 6.833, 6.836, 6.84, 6.845, 6.852, 6.859, 6.867, 6.876, 6.886},
/* PHOSPHATE1 */ {7.118, 7.087, 7.059, 7.036, 7.024, 7.02, 7.016, 7.013, 7.009, 7.006, 7.003, 7, 6.997, 6.994, 6.992, 6.989, 6.987, 6.977, 6.974, 6.97, 6.965, 6.964, 6.965, 6.968, 6.974, 6.982, 6.992, 7.004, 7.018, 7.034, 7.052},
/* PHOSPHATE2 */ {7.534, 7.5, 7.472, 7.448, 7.436, 7.432, 7.429, 7.425, 7.422, 7.419, 7.416, 7.413, 7.41, 7.407, 7.405, 7.402, 7.4, 7.389, 7.386, 7.38, 7.373, 7.367},
/* PHTHALATE  */ {4, 3.998, 3.997, 3.998, 3.999, 4, 4.001, 4.001, 4.002, 4.003, 4.004, 4.005, 4.006, 4.007, 4.008, 4.009, 4.011, 4.018, 4.022, 4.027, 4.038, 4.05, 4.064, 4.08, 4.097, 4.116, 4.137, 4.159, 4.183, 4.208, 4.235},
/* TARTRATE   */ {3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.557, 3.556, 3.555, 3.554, 3.553, 3.552, 3.549, 3.548, 3.547, 3.547, 3.549, 3.554, 3.56, 3.569, 3.58, 3.593, 3.61, 3.628, 3.65, 3.675},
/* TRIS       */ {8.471, 8.303, 8.142, 7.988, 7.899, 7.869, 7.84, 7.812, 7.783, 7.755, 7.727, 7.699, 7.671, 7.644, 7.617, 7.59, 7.563, 7.433, 7.382, 7.307, 7.186, 7.07},
/* PH4        */ {4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4 },
/* PH10       */ {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10 },
};
// *INDENT-ON*

#define ZERO_POINT_TOLERANCE (0.003)
#define PH_ISO (7)
#define AVOGADRO (8.314)
#define FARADAY_CONSTANT (96485.0)
#define KELVIN_OFFSET (273.1)

class CN0398
{
private:
public:
    CN0398();

    float read_rtd();
    float read_ph(float temperature = 25.0);
    float read_moisture();

    enum {
        P1 = 0,
        P2 = 1,
        P3 = 2,
        P4 = 3
    };

    int32_t read_channel(uint8_t ch);
    float data_to_voltage(uint32_t data, uint8_t gain = 1, float VREF = 2.5);
    float data_to_voltage_bipolar(uint32_t data, uint8_t gain = 1, float VREF = 2.5);
    void enable_channel(int channel);
    void disable_channel(int channel);
    void calibrate_ph_pt0(float temperature = 25.0);
    void calibrate_ph_pt1(float temperature = 25.0);
    void calibrate_ph_offset();
    void enable_current_source0(int current_source_channel);
    void enable_current_source1(int current_source_channel);
    void set_digital_output(uint8_t p, bool state);
    void start_single_conversion();
    void reset();
    void setup();
    void init();
    void set_data(void);
    void display_data(void);
    void calibrate_ph(void);
    void print_calibration_solutions(void);

    AD7124 ad7124;

    bool use_nernst = false;
    const float default_offset_voltage = 0;
    const uint16_t SENSOR_SETTLING_TIME = 400; /*in ms*/
    float offset_voltage;
    float default_calibration_ph[2][2] = {{4, 0.169534}, {10,  -0.134135}};
    float calibration_ph[2][2];
    uint8_t solution0,solution1;

};

/********************************* Configuration****************************************/
#define DISPLAY_REFRESH 1000       //ms
#define USE_PH_CALIBRATION        YES// select to do calibration in two point at init
#define USE_LOAD_FACTOR           NO

#define USE_MANUFACTURER_MOISTURE_EQ  //comment if don't want to use

#define TEMPERATURE_SENSOR_PRESENT
#define MOISTURE_SENSOR_PRESENT
#define PH_SENSOR_PRESENT

//#define USE_LINEAR_TEMP_EQ  // comment if you don't want to use it

#endif
