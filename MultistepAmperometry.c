 #include "hal_types.h"
#include "TI_LMP91000.h"
#include "TI_CC2541_LMP91000_i2c.h"
#include "TI_LMP91000_register_settings.h"
#include "Multistep_AD_settings.h"

unsigned char MultistepAmp(uint32 t_passed, unsigned char STATE)
{
         // TIME0 < t <= TIME1
         if ((t_passed > SENS_TIME0) && (t_passed <= SENS_TIME1) && (STATE != 0x02))
         { 
            LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | BIAS_SIGN_POSITIVE| BIAS_24_PERCENT), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_TEMP_MEAS_TIA_ON));
            STATE = 0x02;
         }
        // TIME1 < t <= TIME2
         else if ((t_passed > SENS_TIME1) && (t_passed <= SENS_TIME2) && (STATE != 0x03))
        { 
            LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | BIAS_SIGN_NEGATIVE | BIAS_24_PERCENT), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_TEMP_MEAS_TIA_ON));
           STATE = 0x03;
        }
         // TIME2 < t <= TIME3
         else if ((t_passed > SENS_TIME2) && (t_passed <= SENS_TIME3) && (STATE != 0x04))
         { 
             LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | BIAS_SIGN_POSITIVE| BIAS_24_PERCENT), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_TEMP_MEAS_TIA_ON));
             STATE |= 0x04;
         } 
        // t > TIME3
         else if (t_passed >= SENS_TIME3  && (STATE != 0x01) )
         { 
           LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | BIAS_SIGN_POSITIVE| BIAS_0_PERCENT), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_DEEP_SLEEP));
           STATE = 0x01; // Reset this guy
//          SAMPLENUM = 0; // Reset sample counter
         }    
         else
         {    
            // Just keep counting
         }
return STATE;
}