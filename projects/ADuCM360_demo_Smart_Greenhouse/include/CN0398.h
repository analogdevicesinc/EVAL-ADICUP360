#ifndef _CN0398_H_
   #define _CN0398_H_

   #define RTD_CHANNEL        0
   #define PH_CHANNEL         1
   #define MOISTURE_CHANNEL   2

   #define R0        100.0

   #define YES       1
   #define NO        0

   #define RREF                           (5000.0)

   #define TEMP_GAIN                      (16.0)
   #define PT100_RESISTANCE_TO_TEMP(x)    ((x-100.0)/(0.385))
   #define _2_23                          (1<<23)

   #define ZERO_POINT_TOLERANCE (0.003)
   #define PH_ISO (7)
   #define AVOGADRO (8.314)
   #define FARADAY_CONSTANT (96485.0)
   #define KELVIN_OFFSET (273.1)

   #define NUMBER_OF_TEMPERATURE_ENTRIES 31

   /****************** Configuration Definitions ******************/
   #define PH_SENSOR_PRESENT
   #define TEMPERATURE_SENSOR_PRESENT
   #define MOISTURE_SENSOR_PRESENT

   #define USE_MANUFACTURER_MOISTURE_EQ

   #define USE_PH_CALIBRATION             YES
   #define USE_LOAD_FACTOR                NO
   /***************************************************************/

   enum
   {
     P1 = 0,
     P2 = 1,
     P3 = 2,
     P4 = 3
   };

   enum
   {
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

   void CN0398_Init(void);
   void CN0398_Setup(void);
   float CN0398_read_rtd(void);
   int32_t CN0398_read_channel(uint8_t ch);
   void CN0398_start_single_conversion(void);
   void CN0398_disable_channel(int channel);
   void CN0398_enable_channel(int channel);
   float CN0398_read_ph(float temperature);
   void CN0398_set_digital_output(uint8_t p, uint8_t state);
   float CN0398_read_moisture(void);
   float CN0398_data_to_voltage_bipolar(uint32_t data, uint8_t gain, float VREF);
   void CN0398_calibrate(void);
   void CN0398_calibrate_ph(void);
   void CN0398_calibrate_ph_offset(void);
   void CN0398_print_calibration_solutions(void);
   void CN0398_calibrate_ph_pt0(float temperature);
   void CN0398_calibrate_ph_pt1(float temperature);
   void CN0398_set_data(void);
   void CN0398_display_data(void);

   extern const char solutions[NUMBER_OF_SOLUTIONS][20];
   extern const uint8_t ph_temperatures[NUMBER_OF_TEMPERATURE_ENTRIES];
   extern const float ph_temp_lut[NUMBER_OF_SOLUTIONS][NUMBER_OF_TEMPERATURE_ENTRIES];

   extern uint8_t use_nernst;

#endif
