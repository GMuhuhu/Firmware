/**************************************************************************************************
  Filename:       sensorBLEBroadcaster_Main.c
  Revised:        $Date: 2012-10-19 $
  Revision:       $Revision:$

  Description:    This file contains the main and callback functions for
                  the sensor BLE Broadcaster sample application.

  Copyright 2011 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS" WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/**************************************************************************************************
 *                                           Includes
 **************************************************************************************************/
/* Hal Drivers */

#include "hal_types.h"
#include "hal_key.h"
#include "hal_timer.h"
#include "hal_drivers.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_i2c.h"

/* OSAL */
#include "OSAL.h"
#include "OSAL_Tasks.h"
#include "OSAL_PwrMgr.h"
#include "osal_snv.h"
#include "OnBoard.h"

#include "sensorBLEPeripheral.h"

#include "TI_LMP91000.h"
#include "TI_CC2541_LMP91000_i2c.h"
#include "TI_LMP91000_register_settings.h"

/* eChem specific */
#include "eChem.h"

    
#if defined O2_SENSOR
  #include "O2_Sensor_Settings.h"
#elif defined CO_SENSOR  
  #include "CO_Sensor_Settings.h"
#elif defined GLUCOSE_SENSOR  
  #include "GLUCOSE_Sensor_Settings.h"
#elif defined OCP_SENSOR
  #include "OCP_Sensor_Settings.h"
#elif defined CSWV_SENSOR
  #include "CSWV_Sensor_Settings.h"
#elif defined MULTISTEP_AD
  #include "Multistep_AD_settings.h"
  #include "MultistepAmperometry.h"
#elif defined CHLORIDE_SENSOR
  #include "Chloride_Sensor_Settings.h"
#endif

#ifdef SPI
  /***********************************************************************************
  * INCLUDES
  */
  #include <hal_types.h>
  // Include Name definitions of individual bits and bit-fields in the CC254x device registers.
  #include <ioCC254x_bitdef.h>
  // Include device specific file
  #include "ioCC2541.h"

  /***********************************************************************************
  * CONSTANTS
  */

  // These values will give a baud rate of approx. 2.00 Mbps at 32 MHz system clock
  #define SPI_BAUD_M  0
  #define SPI_BAUD_E  16

  // Define size of buffer and number of bytes to send
  #define BUFFER_SIZE 6

  /***********************************************************************************
  * LOCAL VARIABLES
  */

  // Masters's transmit buffer
  static uint8 txBufferMaster[BUFFER_SIZE];
#endif


/*********************************************************************
 * CONSTANTS
 */

// How often to collect sensor data periodic event
#define DEFAULT_DATA_COLLECT_PERIOD           POLL_PERIOD

// How many seconds to wait between SWV
#define SWV_DELAY          60

// How many scans to perform each CSWV
#define SWV_NUM_CYCLES      2

// SWV potential table
#define SWV_TABLESIZE  30
unsigned char SWV_table[] =
{
   (BIAS_SIGN_NEGATIVE | BIAS_8_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_4_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_6_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_2_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_4_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_0_PERCENT),
   (BIAS_SIGN_NEGATIVE | BIAS_2_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_2_PERCENT), 
   (BIAS_SIGN_POSITIVE | BIAS_0_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_4_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_2_PERCENT), 
   (BIAS_SIGN_POSITIVE | BIAS_6_PERCENT), 
   (BIAS_SIGN_POSITIVE | BIAS_4_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_8_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_6_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_10_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_8_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_12_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_10_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_14_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_12_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_16_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_14_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_18_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_16_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_20_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_18_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_22_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_20_PERCENT),
   (BIAS_SIGN_POSITIVE | BIAS_24_PERCENT),
};


//#define SWV_TABLESIZE  28
//unsigned char SWV_table[] =
//{
//   (BIAS_SIGN_NEGATIVE | BIAS_14_PERCENT), // -420
//   (BIAS_SIGN_NEGATIVE | BIAS_10_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_12_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_8_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_10_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_6_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_8_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_4_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_6_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_2_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_4_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_0_PERCENT),
//   (BIAS_SIGN_NEGATIVE | BIAS_2_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_2_PERCENT), 
//   (BIAS_SIGN_POSITIVE | BIAS_0_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_4_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_2_PERCENT), 
//   (BIAS_SIGN_POSITIVE | BIAS_6_PERCENT), 
//   (BIAS_SIGN_POSITIVE | BIAS_4_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_8_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_6_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_10_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_8_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_12_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_10_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_14_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_12_PERCENT),
//   (BIAS_SIGN_POSITIVE | BIAS_16_PERCENT), // 480
//};


/**************************************************************************************************
 * EXTERN GLOBAL VARIABLES
 **************************************************************************************************/
extern uint8 DataReadyFlag;
    

/**************************************************************************************************
 * FUNCTIONS
 **************************************************************************************************/

/* This callback is triggered when a key is pressed */
void MSA_Main_KeyCallback(uint8 keys, uint8 state);

/**************************************************************************************************
   * @fn          main
 *
 * @brief       Start of application.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
int main(void)
{
  /* Initialize hardware */
  HAL_BOARD_INIT();

  // Initialize board I/O
  InitBoard( OB_COLD );

  /* Initialze the HAL driver */
  HalDriverInit();

  /* Initialize NV system */
  osal_snv_init();
  
  /* Initialize the operating system */
  osal_init_system();

  /* Enable interrupts */
  HAL_ENABLE_INTERRUPTS();
  
  
  #ifdef SPI
   // SPI setup
  /****************************************************************************
     * Clock setup
     * See basic software example "clk_xosc_cc254x"
     */
    
    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed
    
    // Note the 32 kHz RCOSC starts calibrating, if not disabled.


    /***************************************************************************
     * Setup I/O ports
     *
     * Port and pins used by USART0 operating in SPI-mode are
     * MISO (MI): P0_2
     * MOSI (MO): P0_3
     * SSN (SS) : P0_4
     * SCK (C)  : P0_5
     *
     * These pins can be set to function as peripheral I/O to be be used by
     * USART0 SPI. Note however, when SPI is in master mode, only MOSI, MISO,
     * and SCK should be configured as peripheral I/O's. If the external
     * slave device requires a slave select signal (SSN), then the master
     * can control the external SSN by using one of its GPIO pin as output.
     */
    

    // Configure USART0 for Alternative 1 => Port P0 (PERCFG.U0CFG = 0).
    PERCFG = (PERCFG & ~PERCFG_U0CFG) | PERCFG_U0CFG_ALT1;    
        

    // Give priority to USART 0 over Timer 1 for port 0 pins.
     P2DIR &= P2DIR_PRIP0_USART0;


    // Set pins 2, 3 and 5 as peripheral I/O and pin 4 as GPIO output.
    P0SEL = (P0SEL & ~BIT4) | BIT5 | BIT3 | BIT2;
    P0DIR |= BIT4;

//    // Configure P1_0 as GPIO output for LED1.
//    P1SEL &= BIT0;      // GPIO.
//    P1DIR |= BIT0;      // Output.
//    P1_0 = 0;           // LED1 off.

    
    /***************************************************************************
     * Configure SPI
     */

//    // Fill array with bytes to send.
//    uint8 value = 0x00;
//    int i;
//    for (i = 0; i < BUFFER_SIZE; i++)
//    {
//        txBufferMaster[i] = value++;
//    }

    // Set USART to SPI mode and Master mode.
     U0CSR &= ~(U0CSR_MODE | U0CSR_SLAVE);

    // Set:
    // - mantissa value
    // - exponent value
    // - clock phase to be centered on first edge of SCK period
    // - negative clock polarity (SCK low when idle)
    // - bit order for transfers to LSB first
    U0BAUD = SPI_BAUD_M;
    U0GCR = (U0GCR & ~(U0GCR_BAUD_E | U0GCR_CPOL | U0GCR_CPHA | U0GCR_ORDER))
        | SPI_BAUD_E;
#endif

  // Final board initialization
  InitBoard( OB_READY );
 P1DIR = P1DIR | 0x01; 
        P1 = P1 | 0x01;
  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
    //osal_pwrmgr_powerconserve();
     
  #endif
  /* Start OSAL */
  osal_start_system(); // No Return from here
  



  return 0;
} // end main()

/**************************************************************************************************
                                           CALL-BACKS
**************************************************************************************************/


/*************************************************************************************************
**************************************************************************************************/
// Set COMPUTE_91K_TEMP_WITH_TABLE to 1 to use a table lookup (scan) to convert
// from millivolts to degrees C.  If COMUTE_91K_TEMP_WITH_TABLE is 0, we'll 
// do a fit to a pre-computed curve
#define COMPUTE_91K_TEMP_WITH_TABLE 0

#if COMPUTE_91K_TEMP_WITH_TABLE
int tempTable[] =
{
 1875,  /* 1875 mv at -40 C */
 1867, 
 1860, 
 1852, 
 1844, 
 1836, 
 1828, 
 1821, 
 1813, 
 1805, 
 1797, 
 1789, 
 1782, 
 1774, 
 1766, 
 1758, 
 1750, 
 1742, 
 1734, 
 1727, 
 1719, 
 1711, 
 1703, 
 1695, 
 1687, 
 1679, 
 1671, 
 1663, 
 1656, 
 1648, 
 1640, 
 1632, 
 1624, 
 1616, 
 1608, 
 1600, 
 1592, 
 1584, 
 1576, 
 1568, 
 1560, /* 0 */
 1552, 
 1544, 
 1536, 
 1528, 
 1520, 
 1512, 
 1504, 
 1496, 
 1488, 
 1480, 
 1472, 
 1464, 
 1456, 
 1448, 
 1440, 
 1432, 
 1424, 
 1415, 
 1407, 
 1399, 
 1391, 
 1383, 
 1375, 
 1367, 
 1359, 
 1351, 
 1342, 
 1334, 
 1326, 
 1318, 
 1310, 
 1302, 
 1293, 
 1285, 
 1277, 
 1269, 
 1261, 
 1253, 
 1244, 
 1236, 
 1228, 
 1220, 
 1212, 
 1203, 
 1195, 
 1187, 
 1179, 
 1170, 
 1162, 
 1154, 
 1146, 
 1137, 
 1129, 
 1121, 
 1112, 
 1104, 
 1096, 
 1087, 
 1079, 
 1071, 
 1063, 
 1054, 
 1046, 
 1038, 
 1029, 
 1021, 
 1012, 
 1004, 
 996, 
 987, 
 979, 
 971, 
 962, 
 954, 
 945, 
 937, 
 929, 
 920, 
 912, 
 903, 
 895, 
 886, 
 878, 
 870, 
 861, /* 85 */
};
#endif

/**************************************************************************************************
 * @fn          convertTemp
 *
 * @brief       Convert tempval to tenths of degrees C, based on the table in the datasheet for the LMP91000
 *
 * @param       int16 original value
 *
 * @return      int16 converted value
 **************************************************************************************************
 */
int16 convertTemp(int16 tempval)
{
  int16 newtemp;
  int16 millivolts;
#if COMPUTE_91K_TEMP_WITH_TABLE
  int index;
#else
  // No extra variables needed for the curve-fit
#endif
  
  // Clamp at 0, shouldn't be negative
  if (tempval <0) 
  {
    tempval = 0; 
  }
  
  // Convert to millivolts, assuming 12 bit conversion (2047 max code)
  // and a 2.5 volt external reference

  millivolts = (int16) ((tempval * ECHEM_VDD*1000) / 2047.0);


#if COMPUTE_91K_TEMP_WITH_TABLE  
  // The table is -close- to linear, but not quite, so just walk it to find 
  // the range of values of interest.
  index = 0; // -40C
  while ((index<40+85) && (tempTable[index]>millivolts) )
  {
    index++;
  }
  // At this point, index is indicating the table element that is
  // greater than the voltage reading (unless index==0)
  // If we ran past the end of the table (85 degrees), index
  // will be pointing to the last element of the table (close enough).
  if (index==0)
  {
    // fudge it, and bump the index by 1
    index++;
  }
  
  newtemp = (index-41)*10;
#else
 
  
        newtemp = (int16) (300.0 + (tempval/4.5));

    // Instead of the table lookup above, here's a good answer
      // TinDegreesCelsius = millivolts * (millivolts * A + B) + C
      // where
      // A = -5.23385 x 10-6
      // B = -0.108906
      // C = 182.635
    #define C_val (182.635)
    #define B_val (-0.108906)
    #define A_val (-5.23385E-6)
      newtemp = (int16) (10.0 * ( ((float) millivolts) * ( ( ( (float) millivolts) * A_val) + (B_val)) + C_val));
#endif  

  return(newtemp);
} // end convertTemp()
       




static uint8 DataUpdate_TaskID;   // Task ID for internal task/event processing


/**************************************************************************************************
 * @fn          DataUpdateTask_Init
 *
 * @brief       Set up task and variables for the data update task
 *
 * @param       uint8 task_id
 *
 * @return      none
 **************************************************************************************************
 */
void DataUpdateTask_Init( uint8 task_id )
{
  DataUpdate_TaskID = task_id;
  
  //                                          event_id   ms
  osal_start_reload_timer (DataUpdate_TaskID,    0x1,   DEFAULT_DATA_COLLECT_PERIOD);
} // end DataUpdateTask_Init()

static int16 oldtempval;   // old value read from temp sensor
static int16 oldsensorval; // old value read from O2 sensor
static int8 cyclecount;    // used for operating the LED
static int16 SAMPLENUM; // Sample counter
static int16 CYCLENUM = 1000;; //
static uint16 t_passed;
static unsigned char STATE = 0x00;// Mask for MA state
static unsigned char PREV_STATE = 0x01;// Set PREV_STATE different from STATE
unsigned char FORWARD_SCAN = 0x01;


/**************************************************************************************************
 * @fn          DataUpdate_ProcessEvent
 *
 * @brief       Update values from A/D sensor and diags
 *
 * @param       uint8 task_id
 * @param       uint16 events - event mask
 *
 * @return      uint16 - 0 (for now)
 **************************************************************************************************
 */
uint16 DataUpdate_ProcessEvent(uint8 task_id, uint16 events)
{
    #if ADV_DEBUG_MESSAGE_FORMAT==1
    // variables that are needed if we're running the advanced debugging message format
    int vdd_div_3;
    int vdd;
//    int16 tempvalCC2541;
    int spare;
    #endif
    
    int tempval;
    int16 tempvalCC2541;  // Moved here to use in OCP mode
    int16 tempval2;
    int16 tempval3;
    int16 tempval4;
    int16 tempavg;
    uint16 timeval;
    uint16 sensorval;
    

//#if USE_SEPARATE_TEMP_AD_CHANNEL==0
//    uint8 lmp_configured;
//    uint8 lmp_configure_tries;
//#endif

#ifdef FAKE_SENS_DATA

#ifdef O2_SENSOR
    uint16 max_fake = 1000;
    uint16 min_fake = 600;
    static int16  fake_adj = 1;
    static uint16 fakesensorval = 600;
#endif
#ifdef CO_SENSOR
    uint16 max_fake = 1300;
    uint16 min_fake = 380;
    static int16  fake_adj = 1;
    static uint16 fakesensorval = 380;
#endif
#ifdef GLUCOSE_SENSOR || CHLORIDE_SENSOR
    uint16 max_fake = 600;
    uint16 min_fake = 400;
    static int16  fake_adj = 5;
    static uint16 fakesensorval = 500;
#endif
#ifdef OCP_SENSOR
    uint16 max_fake = 100;
    uint16 min_fake = 0;
    static int16  fake_adj = 1;
    static uint16 fakesensorval = 50;
#endif
#ifdef MULTISTEP_AD
    uint16 max_fake = 100;
    uint16 min_fake = 0;
    static int16  fake_adj = 1;
    static uint16 fakesensorval = 50;
#endif

    
#endif    
    if (events & 1)
    {
      timeval = (uint16) (osal_GetSystemClock() & 0xffff);
      cyclecount++;
      if (cyclecount>9)
      {
        cyclecount=0;
        // Also, set P1_0 (the LED) as an output, and drive high
        P1DIR = P1DIR | 0x01; 
        P1 = P1 | 0x01;
      }
      
      // Just enable all channels for debugging (SS Jan-2018)
         ADCCFG |= 0xC2; // Ad
         P0DIR |= 0x43; // force P0.0, P0.1 and P0.6
         APCFG = 0x43;
         
#ifdef OCP_SENSOR
         P1 |= 0x02; // Drive HIGH to enable MAX4461
         HalAdcSetReference (0);
          P1 = P1&0xFD;  // Drive LOW to disable MAX4461
#else
         HalAdcSetReference (HAL_ADC_REF_AVDD);
#endif

  
#ifdef MULTISTEP_AD
  #if defined ( POWER_SAVING )
    osal_pwrmgr_device(  PWRMGR_ALWAYS_ON);
  #endif

  SAMPLENUM++;
  t_passed = SAMPLENUM*DEFAULT_DATA_COLLECT_PERIOD/1000;

  STATE = MultistepAmp(t_passed,STATE);
  
      if ((PREV_STATE != 0x01) && ( STATE == 0x01))
    {
    SAMPLENUM = 0;
    }
  PREV_STATE = STATE;
  #if defined ( POWER_SAVING )
    osal_pwrmgr_device( PWRMGR_BATTERY );
  #endif
#endif
      


// It is not possible to turn on the temperature sensor of the LMP91000 without turning on A1 and polarizing the electrodes
      
#ifdef OCP_SENSOR // Read temp from CC2541 instead
//            HalAdcSetReference(HAL_ADC_REF_AVDD); // use internal ref voltage (1.15 V)
         //HalAdcSetReference ( 0x80); // use AVDD5 pin for ref
         
         // CC2541 Internal temp sensor is A/D input 14 (0x0e)
         tempvalCC2541 = HalAdcRead(0x0E, HAL_ADC_RESOLUTION_12); 
//        tempval = (int16) (300.0 + (tempvalCC2541/4.5));
        tempval = (int16) (124.0 + (tempvalCC2541/2.41)); // 2.41 is the sensitivity with 1.24 as reference voltage
        
//        HalAdcSetReference(HAL_ADC_REF_125V); //use internal ref voltage (1.24 V)

#else
       // Read temperature from LMP9100
       if (!lmp91kOK)
       {
         // If we're not communicating, then the LMP91000 may not be in the right
         // state, so just use a previous value.
         tempval = oldtempval;
       }
       else
       {
         tempval =  HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
         tempval2 =  HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
         tempval3 =  HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
         tempval4 =  HalAdcRead(HAL_ADC_CHANNEL_0, HAL_ADC_RESOLUTION_12);
         // Get a bit of noise out of the temperature measurment by averaging 4
         // samples.
         // By the way, we expect nominal values to be around 1100 or so
         // at room temperature, so we can fairly safely add 4 16-bit values
         // together without thinking too much about overflow.
         // If the nominal values were larger, we'd promote to a larger
         // data type before averaging.
         tempavg = (tempval+tempval2+tempval3+tempval4)/4;
         oldtempval=tempavg;
       }
       
       
       // Convert temp to tenths of degrees C, based on the table in
       // the datasheet for the LMP91000, and assuming a 2.5v ref
       
       tempval = convertTemp(tempavg);

       // Now, get ready to measure the oxygen sensor
       HalAdcSetReference (HAL_ADC_REF_AVDD); // use AIN7 for ref or 0x00 for internal
#endif
       
       
       
        
       
         #ifdef OCP_SENSOR // Read from MAX4461
//           HalAdcSetReference(HAL_ADC_REF_125V);
            int readIDX = 0;
         sensorval =0;
            while (readIDX<3)
            {
              readIDX++;
                     P1 |= 0x02; // Drive HIGH to enable MAX4461
//                       sensorval =  HalAdcRead(HAL_ADC_CHANNEL_6,HAL_ADC_RESOLUTION_12);
            //              sensorval =  HalAdcRead(HAL_ADC_CHN_VDD3,HAL_ADC_RESOLUTION_12);
                      
                      sensorval = sensorval + HalAdcRead(HAL_ADC_CHANNEL_6,HAL_ADC_RESOLUTION_12);
//                       P1 = P1&0xFD;  // Drive LOW to disable MAX4461
            }
            sensorval = sensorval/3;
            
        #else // Voltammetric mode
         {
           if (!lmp91kOK)
           {
             // If we're not communicating, then the LMP91000 may not be in the right
             // state, so just use a previous value.
             sensorval = oldsensorval;
           }

         else
         {

                 sensorval =  HalAdcRead(HAL_ADC_CHANNEL_1,HAL_ADC_RESOLUTION_12);
                 #ifdef CSWV_SENSOR
                   if (CYCLENUM <= SWV_NUM_CYCLES)
                   {
                     #if defined ( POWER_SAVING )
                      osal_pwrmgr_device(  PWRMGR_ALWAYS_ON);
                    #endif
                   
                   LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | SWV_table[SAMPLENUM]), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_TEMP_MEAS_TIA_ON));

                   if (SAMPLENUM == SWV_TABLESIZE-1)
                   {
                     FORWARD_SCAN = 0x00;
                   }
                   if(SAMPLENUM == 0)
                   {
                     FORWARD_SCAN = 0x01;
                     CYCLENUM++;
                   }
                   
                   if (FORWARD_SCAN==0x01)
                   {     
                     SAMPLENUM++;
                   }
                   else
                   {
                     SAMPLENUM--;
                   }
                     #if defined ( POWER_SAVING )
                      osal_pwrmgr_device( PWRMGR_BATTERY );
                    #endif
                   }
                   else
                   {
                        LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | SWV_table[SAMPLENUM]), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_DEEP_SLEEP));
                        SAMPLENUM++;
                        t_passed = (uint16) SAMPLENUM*DEFAULT_DATA_COLLECT_PERIOD/1000;
                        if (t_passed >= SWV_DELAY)
                        {
                         CYCLENUM = 0;
                         SAMPLENUM = 0;
                        }
                        
                   }
                     
                #endif
                   
                #ifdef CHLORIDE_SENSOR

                   SAMPLENUM++;
                   t_passed = (uint16) SAMPLENUM*DEFAULT_DATA_COLLECT_PERIOD/1000;
                   unsigned char CHLORIDE_table[] =                              
                              {
                               (BIAS_SIGN_POSITIVE | BIAS_0_PERCENT),
                               (BIAS_SIGN_POSITIVE | BIAS_1_PERCENT),
                               (BIAS_SIGN_NEGATIVE | BIAS_1_PERCENT),
                              };
                   
                   double uCurrent = 100*(((double)sensorval *(SENS_NUMER_0/ SENS_DENOM_0) - (SENS_SUB_0/100))/SENS_DENOM_1)*(SENS_SCALE_FACTOR_NUM/SENS_SCALE_FACTOR_DENOM);
                   if ((STATE ==0) && ((uCurrent <= SENS_TOP_ZERO_LIMIT) && uCurrent >= SENS_BOT_ZERO_LIMIT) || (STATE !=0)) // Wait intil threshold current is passed   
                   {
                   if (t_passed >= 1) 
                    {
                      STATE++;
                       if (STATE > 2)
                       {
                         STATE = 0;
                       }
                      LMP91000_I2CInitialSetup(SENS_FEEDBACK_GAIN , SENS_RLOAD, (SENS_REF_SOURCE | CHLORIDE_table[STATE]), SENS_INT_Z_REF_DIVIDER, (SENS_FET_SHORT | OP_MODE_TEMP_MEAS_TIA_ON));
                      SAMPLENUM = 0;         
                                              
                    }
                   }
                   

                 //   HalI2CEnterSleep(); 
                #endif

           oldsensorval = sensorval;
         }
        }
 #endif 
       
       // Depending on compile options, build the message, or gather
       // additional diagnostic info and then build the debug mode message
#ifdef FAKE_SENS_DATA
fakesensorval += fake_adj;
if ((fakesensorval >= max_fake) || (fakesensorval <= min_fake))
  fake_adj = -1 * fake_adj;
sensorval = fakesensorval;
//sensorval =  HalAdcRead(HAL_ADC_CHANNEL_1,HAL_ADC_RESOLUTION_12);
#endif
     
       #if ADV_DEBUG_MESSAGE_FORMAT==0
  
          // Set max value to indicate idle/OCP mode in CSWV mode
          #ifdef CSWV_SENSOR
              if (t_passed < SWV_DELAY)
              {
                sensorval=(uint16) 2047;
              }
          #endif 
              
          #ifdef SPI 
             // SPI transfer data
                // Clear SSN, the SPI slave is active when SSN is low.
                P0_4 = 0;
                
                // Fill buffer with data
                txBufferMaster[0] = (uint8)(timeval & 0xff);
                txBufferMaster[1] = (uint8)(timeval >> 8);
                txBufferMaster[2] = (uint8)(tempval & 0xff);
                txBufferMaster[3] = (uint8)(tempval >> 8);
                txBufferMaster[4] = (uint8)(sensorval & 0xff);
                txBufferMaster[5] = (uint8)(sensorval >> 8);
                 int i;
                for (i = 0; i < BUFFER_SIZE; i++)
                {
                    // Write byte to USART0 buffer (transmit data).
                    U0DBUF = txBufferMaster[i];

                    // Check if byte is transmitted.
                    while(!(U0CSR & U0CSR_TX_BYTE));

                    // Clear transmit byte status.
                    U0CSR &= ~U0CSR_TX_BYTE;
                }

                // Set SSN, the SPI slave is inactive when SSN is high.
                P0_4 = 1;
            #endif  
              
              // BLE send
             updateSensorData ((uint16)timeval, (uint16)tempval, (uint16)sensorval);
             DataReadyFlag = 1;
             


//            #if (chip==2541 || chip==2543 || chip==2545)
//                // When finished receiving, set LED1.
//                P1_0 = 1;   // SRF05EB LED1 on.
//            #endif
            
       #else
         // Gather additional interesting diagnostic info before updating structure
         spare = HalAdcRead(0x01, HAL_ADC_RESOLUTION_12); // Spare A/D chan - use for battery measurement later
  
         // Turn on the test mode to enable temperature measurement 
         // from the CC2544's internal temp sensor
         ATEST=1; // ATEST.ATEST_CTRL=0x01;
         TR0=1;   //  TR0.ADCTM=1;
        
         HalAdcSetReference(0); // use internal ref voltage (1.15 V)
         //HalAdcSetReference ( 0x80); // use AVDD5 pin for ref
         
         // CC2541 Internal temp sensor is A/D input 14 (0x0e)
         tempvalCC2541 = HalAdcRead(0x0E, HAL_ADC_RESOLUTION_12); 

         /* Turn off test modes */
         TR0=0; //        TR0.ADCTM=0;
         ATEST=0;

        // The analog temperature sensor in the CC2544 should give back a value 
        // of 1480 at 25 degrees C and VDD=3, 
        // and will change by a value of 4.5 per degree C
        //     
        // So, to get temperature in 0.1 degrees C units, consider the 
        // following formula:
        //  
        tempval2 = tempvalCC2541;
        tempvalCC2541 = (int16) (300.0 + (tempval2/4.5));
      
        HalAdcSetReference(0); // use internal ref voltage (1.15 V)
          
        // Pick up VDD divided by 3
        vdd_div_3 = HalAdcRead(0x0F, HAL_ADC_RESOLUTION_12); // VDD/3
        // Convert to millivolts (and get rid of the divide by 3, since we're doing math
        // vdd = (int) (1.15*vdd_div_3*3.0*1000.0 / 2047.0); // convert to millivolts
        // vdd = (int) (vdd_div_3 * 1.6853932584269662921348314606742); // more precisely
        vdd = (int) (vdd_div_3 * 1.6853); // close enough
        
        // Pick up the spare A/D input
        HalAdcSetReference (0x00); // use AIN7 for ref
        spare = HalAdcRead(0x01, HAL_ADC_RESOLUTION_12); 

        updateSensorData ((uint16)timeval, (uint16)tempval, (uint16)sensorval, (uint16)tempvalCC2541, (uint16)vdd, (uint16)spare);
        DataReadyFlag = 1;
        
        #endif
        
        // Also, set P1_0 (the LED) as an output, and drive low  
        P1DIR = P1DIR | 0x01; 
        P1 = P1&0xFE;

        //halSleep(POLL_PERIOD/2);
        return (events ^ 1);;
        
    }
    
    return (0);
} // end DataUpdate_ProcessEvent()
