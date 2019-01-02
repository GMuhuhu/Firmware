#ifndef __GLUCOSE_SENSOR_SETTINGS_H
#define __GLUCOSE_SENSOR_SETTINGS_H

#include "TI_LMP91000.h"
#include "eChem.h"

// Polling period in ms
#define POLL_PERIOD                             1000

// Initial values for the algorithm used to convert ADC output into meaningful value
#define SENS_DENOM_0                            204800        // 2048 x 100  -- ADC resolution X
#define SENS_NUMER_0                            ECHEM_VDD * 100           // Voltage reference V x 100    
#define SENS_SUB_0                              ECHEM_VDD*0.20*100           
#define SENS_DENOM_1                            3500000        // R_TIA x 100
#define SENS_SCALE_FACTOR_NUM                   -1000000   // Convert into A into uA
#define SENS_SCALE_FACTOR_DENOM                 1            // 70nA per ppm


// LMP91000 Settings for the specific sensor
#define SENS_OPERATIONAL_MODE                   OP_MODE_TEMP_MEAS_TIA_ON
#define SENS_FEEDBACK_GAIN                      TIA_GAIN_35K_OHM
#define SENS_RLOAD                              R_LOAD_10_OHM
#define SENS_INT_Z_REF_DIVIDER                  INT_Z_SEL_20_PERCENT
#define SENS_REF_SOURCE                         REF_SOURCE_INTERNAL
#define SENS_BIAS_SIGN                          BIAS_SIGN_POSITIVE
#define SENS_BIAS_VOLTAGE                       BIAS_22_PERCENT
#define SENS_FET_SHORT                          FET_SHORT_DISABLED                      

// Parameters used by iPhone/iPad app for graphing data
#define SENS_Y_DISPLAY_MAX                      2048
#define SENS_Y_DISPLAY_MIN                      1
#define SENS_GRAPH_TOP_MID_BOUNDARY             1800           // About 20% - Green - Yellow boundary
#define SENS_GRAPH_MID_LOW_BOUNDARY             200           // About 19% - Yellow - Red boundary
#define SENS_GRAPH_TOP_COLOR                    0x1B9E77      // Purple
#define SENS_GRAPH_MID_COLOR                    0x7075B3      // Green
#define SENS_GRAPH_LOW_COLOR                    0xD95F02      // Orange
#define SENS_CALIB                              0    // No Calibration
#define SENS_TYPE                               GLUCOSE_SENSOR_TYPE    
#define SENS_DISPLAY_CURRENT_VALUE              1            // Display current value
#define SENS_DISPLAY_LOG_SCALE                  0             // Do not use logarithmic scale


// Note that 19 character maximum for Characteristic strings, that is 18 characters + \0!!!
#define SENS_GRAPH_TITLE                        "Glucose data\0"
#define SENS_GRAPH_TITLE_SIZE                   13
#define SENS_GRAPH_SUBTITLE                     "Using the LMP91000\0"
#define SENS_GRAPH_SUBTITLE_SIZE                19
#define SENS_GRAPH_X_AXIS_CAPTION               "Time [s] \0"
#define SENS_GRAPH_X_AXIS_CAPTION_SIZE          10
#define SENS_GRAPH_Y_AXIS_CAPTION               "Current [uA]\0"    
#define SENS_GRAPH_Y_AXIS_CAPTION_SIZE          13
#define SENS_SHORT_CAPTION_VALUE                "CA\0"
#define SENS_SHORT_CAPTION_SIZE                 3

#endif // __GLUCOSE_SENSOR_SETTINGS_H
