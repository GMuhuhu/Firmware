#ifndef __OCP_SENSOR_SETTINGS_H
#define __OCP_SENSOR_SETTINGS_H

#include "TI_LMP91000.h"
#include "eChem.h"

// Polling period in ms
#define POLL_PERIOD                             5000

// Initial values for the algorithm used to convert ADC output into meaningful value
#define SENS_DENOM_0                            204800        // 2048 x 100  -- ADC resolution X
#define SENS_NUMER_0                            1.27 * 100           // Voltage reference V x 100    
#define SENS_SUB_0                              0            
#define SENS_DENOM_1                            100        // R_TIA x 100
#define SENS_SCALE_FACTOR_NUM                   1000   // Convert into mv from V
#define SENS_SCALE_FACTOR_DENOM                 -1            // Calibration constant


// LMP91000 Settings for the OCP Sensor
#define SENS_OPERATIONAL_MODE                   OP_MODE_DEEP_SLEEP // Lowest power mode that still has temperature but does not bias electrodes
#define SENS_FEEDBACK_GAIN                      TIA_GAIN_2_75K_OHM // Try to increase input impedance as mucch as possible
#define SENS_RLOAD                              R_LOAD_100_OHM // Try to separate RE and WE as much as possible
#define SENS_INT_Z_REF_DIVIDER                  INT_Z_SEL_20_PERCENT // Does not really matter what this is set to
#define SENS_REF_SOURCE                         REF_SOURCE_INTERNAL // Should cause no biasing of the electrodes
#define SENS_BIAS_SIGN                          BIAS_SIGN_NEGATIVE // Does not really matter what this is set to
#define SENS_BIAS_VOLTAGE                       BIAS_0_PERCENT  // Percentage of a floating input
#define SENS_FET_SHORT                          FET_SHORT_DISABLED

// Parameters used by iPhone/iPad app for graphing data
#define SENS_Y_DISPLAY_MAX                      2047
#define SENS_Y_DISPLAY_MIN                      0
#define SENS_GRAPH_TOP_MID_BOUNDARY             800           // About 20% - Green - Yellow boundary
#define SENS_GRAPH_MID_LOW_BOUNDARY             200           // About 19% - Yellow - Red boundary
#define SENS_GRAPH_TOP_COLOR                    0x1B9E77      // Green
#define SENS_GRAPH_MID_COLOR                    0x7075B3      // Purple
#define SENS_GRAPH_LOW_COLOR                    0xD95F02      // Orange
#define SENS_CALIB                              0    // No Calibration
#define SENS_TYPE                               OCP_SENSOR_TYPE    
#define SENS_DISPLAY_CURRENT_VALUE              1            // Display current value
#define SENS_DISPLAY_LOG_SCALE                  0             // Do not use logarithmic scale


// Note that 19 character maximum for Characteristic strings, that is 18 characters + \0!!!
#define SENS_GRAPH_TITLE                        "Chronopotentiometry\0"
#define SENS_GRAPH_TITLE_SIZE                   20
#define SENS_GRAPH_SUBTITLE                     "Using the MAX4461\0"
#define SENS_GRAPH_SUBTITLE_SIZE                18
#define SENS_GRAPH_X_AXIS_CAPTION               "Time / s \0"
#define SENS_GRAPH_X_AXIS_CAPTION_SIZE          10
#define SENS_GRAPH_Y_AXIS_CAPTION               "Potential / mV \0"    
#define SENS_GRAPH_Y_AXIS_CAPTION_SIZE          16
#define SENS_SHORT_CAPTION_VALUE                "OCP\0"
#define SENS_SHORT_CAPTION_SIZE                 4

#endif // __OCP_SENSOR_SETTINGS_H
