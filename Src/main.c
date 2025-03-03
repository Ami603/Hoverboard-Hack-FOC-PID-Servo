/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
* Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdio.h>
#include <stdlib.h> // for abs()
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"      /* BLDC's header file */
#include "rtwtypes.h"

void SystemClock_Config(void);

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------


extern TIM_HandleTypeDef htim_left;
extern TIM_HandleTypeDef htim_right;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern volatile adc_buf_t adc_buffer;


extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

volatile uint8_t uart_buf[200];

// Matlab defines - from auto-code generation
//---------------
extern P    rtP_Left;                   /* Block parameters (auto storage) */
extern P    rtP_Right;                  /* Block parameters (auto storage) */
extern ExtY rtY_Left;                   /* External outputs */
extern ExtY rtY_Right;                  /* External outputs */
//---------------

extern uint8_t     inIdx;               // input index used for dual-inputs
extern InputStruct input1[];            // input structure
extern InputStruct input2[];            // input structure

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern volatile uint32_t timeoutCntGen; // Timeout counter for the General timeout (PPM, PWM, Nunchuk)
extern volatile uint8_t  timeoutFlgGen; // Timeout Flag for the General timeout (PPM, PWM, Nunchuk)
extern uint8_t timeoutFlgADC;           // Timeout Flag for for ADC Protection: 0 = OK, 1 = Problem detected (line disconnected or wrong ADC data)
extern uint8_t timeoutFlgSerial;        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwml;               // global variable for pwm left. -1000 to 1000
extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000


extern uint8_t enable;                  // global variable for motor enable

extern int16_t batVoltage;              // global variable for battery voltage



_Bool update_periodic_loop( volatile uint32_t loop_ms); //vam added to create accurate periodic loop


extern void Motor_Pos();                //vam calculate cumulation of encoder positions left and right

#if defined(SIDEBOARD_SERIAL_USART2)
extern SerialSideboard Sideboard_L;
#endif
#if defined(SIDEBOARD_SERIAL_USART3)
extern SerialSideboard Sideboard_R;
#endif



//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
int motAngleLeft = 0;
int motAngleRight = 0;
int MotorPosL = 0;
int MotorPosR = 0;
uint8_t backwardDrive;
volatile uint32_t main_loop_counter;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
uint32_t loop_time = 0; //Vam added to measure delays in loop
uint32_t tick_last = 0; 
#ifdef PID_CONTROL   //Vam added for closed loop control of motors using HAL sensors


PID_DATA PIDL ={KP,KI,PIDDZ,PIDLIM,0,0,0,0,0}; //{KP,KI,PIDDZ,PIDLIM, 0,0,0};  //pid sturcture for left wheel
PID_DATA PIDR ={KP,KI,PIDDZ,PIDLIM,0,0,0,0,0};  //pid structure for right wheel
#endif


#if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)
typedef struct{
  uint16_t  start;
  int16_t   cmd1;
  int16_t   cmd2;
  int16_t   speedR_meas;
  int16_t   speedL_meas;
  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t  cmdLed;
  uint16_t  checksum;
} SerialFeedback;
static SerialFeedback Feedback;

typedef struct{
   uint16_t  start;
   int16_t   steer;
   int16_t   speed;
   uint16_t  checksum;
    } SCommand;
static SCommand Command;
#endif
#if defined(FEEDBACK_SERIAL_USART2)
static uint8_t sideboard_leds_L;
#endif
#if defined(FEEDBACK_SERIAL_USART3)
static uint8_t sideboard_leds_R;
#endif

static int16_t    speed;                // local variable for speed. -1000 to 1000
#ifndef VARIANT_TRANSPOTTER
  static int16_t  speed1;                //lodal variable for motorL speed command
  static int16_t  speed2;                //local variable for motorR speed command
  static int16_t  steer;                // local variable for steering. -1000 to 1000
  static int16_t  speed1RateFixdt;       // local fixed-point variable for steed1 rate limiter
  static int16_t  speed2RateFixdt;       // local fixed-point variable for speed2 rate limiter
  static int32_t  speed1Fixdt;           // local fixed-point variable for speed1 low-pass filter
  static int32_t  speed2Fixdt;           // local fixed-point variable for speed2 low-pass filter
#endif

static uint32_t    inactivity_timeout_counter;
static MultipleTap MultipleTapBrake;    // define multiple tap functionality for the Brake pedal



int main(void) {

  HAL_Init();
  __HAL_RCC_AFIO_CLK_ENABLE();
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  /* System interrupt init*/
  /* MemoryManagement_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
  /* BusFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
  /* UsageFault_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
  /* SVCall_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
  /* DebugMonitor_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

  SystemClock_Config();

  __HAL_RCC_DMA1_CLK_DISABLE();
  MX_GPIO_Init();
  MX_TIM_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  BLDC_Init();        // BLDC Controller Init

  //HAL_GPIO_WritePin(SWITCH_PORT, SWITCH_PIN, GPIO_PIN_SET);   // Activate Latch
  //HAL_GPIO_WritePin(OFF_PORT,OFF_PIN, GPIO_PIN_SET);   // Activate Latch this switches board off if open
	
  Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  HAL_ADC_Start(&hadc1);
  HAL_ADC_Start(&hadc2);

  poweronMelody();
  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

  int16_t cmdL      = 0, cmdR      = 0;
  int16_t cmdL_prev = 0, cmdR_prev = 0;

  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;
  int16_t board_temp_deg_c;

  while(HAL_GPIO_ReadPin(BUTTON_PORT,BUTTON_PIN)) HAL_Delay(10);  //keeps from entering auto_cal with long button push
  
	while(1){
	if(update_periodic_loop(DELAY_IN_MAIN_LOOP)) {
   // HAL_Delay(DELAY_IN_MAIN_LOOP);        // delay in ms
    
    readCommand();                        // Read Command: input1[inIdx].cmd, input2[inIdx].cmd
	//printf("commandl low: %d commandr low: %d commandl high:%d commandr high:%d\r\n",input1[inIdx].cmd, input2[inIdx].cmd,input1[inIdx].raw, input2[inIdx].raw);
	// SEND raw high to next controller
	if (main_loop_counter % 2 == 0 && !timeoutFlgSerial)
	{ 
			Command.start    = (uint16_t)SERIAL_START_FRAME;
			Command.steer    = (int16_t) input1[inIdx].raw;
			Command.speed    = (int16_t) input2[inIdx].raw;
			Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);	
			if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0)
			{
				HAL_UART_Transmit_DMA(&huart3, (uint32_t *)&Command, sizeof(Command));
			}
	}
   // if (main_loop_counter % 25 == 0)   {
	//		printf("input1 cmd input1[inIdx].cmd:%i input2[inIdx].cmd:%i,/r/n",input1[inIdx].cmd,input2[inIdx].cmd);}
		
		calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAb		
    
    #if !(defined VARIANT_TRANSPOTTER  || defined PID_CONTROL)
      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 && (!rtY_Left.z_errCode && !rtY_Right.z_errCode) && (input1[inIdx].cmd > -50 && input1[inIdx].cmd < 50) && (input2[inIdx].cmd > -50 && input2[inIdx].cmd < 50)){
        beepShort(6);                     // make 2 beeps indicating the motor enable
        beepShort(4); HAL_Delay(100);
        steerFixdt = speedFixdt = 0;      // reset filters
        enable = 1;                       // enable motors
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }

      // ####### VARIANT_HOVERCAR #######
      #if defined(VARIANT_HOVERCAR) || defined(VARIANT_SKATEBOARD) || defined(ELECTRIC_BRAKE_ENABLE)
        uint16_t speedBlend;                                        // Calculate speed Blend, a number between [0, 1] in fixdt(0,16,15)
        speedBlend = (uint16_t)(((CLAMP(speedAvgAbs,10,60) - 10) << 15) / 50); // speedBlend [0,1] is within [10 rpm, 60rpm]
      #endif

      #ifdef STANDSTILL_HOLD_ENABLE
        standstillHold();                                           // Apply Standstill Hold functionality. Only available and makes sense for VOLTAGE or TORQUE Mode
      #endif

      #ifdef ELECTRIC_BRAKE_ENABLE
        electricBrake(speedBlend, MultipleTapBrake.b_multipleTap);  // Apply Electric Brake. Only available and makes sense for TORQUE Mode
      #endif

      // ####### LOW-PASS FILTER #######
      rateLimiter16(input1[inIdx].cmd , RATE, &steerRateFixdt);
      rateLimiter16(input2[inIdx].cmd , RATE, &speedRateFixdt);
      filtLowPass32(steerRateFixdt >> 4, FILTER, &steerFixdt);
      filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
      steer = (int16_t)(steerFixdt >> 16);  // convert fixed-point to integer
      speed = (int16_t)(speedFixdt >> 16);  // convert fixed-point to integer

      // ####### MIXER #######
      // cmdR = CLAMP((int)(speed * SPEED_COEFFICIENT -  steer * STEER_COEFFICIENT), INPUT_MIN, INPUT_MAX);
      // cmdL = CLAMP((int)(speed * SPEED_COEFFICIENT +  steer * STEER_COEFFICIENT), INPUT_MIN, INPUT_MAX);
      mixerFcn(speed << 4, steer << 4, &cmdR, &cmdL);   // This function implements the equations above

      // ####### SET OUTPUTS (if the target change is less than +/- 100) #######
      if ((cmdL > cmdL_prev-100 && cmdL < cmdL_prev+100) && (cmdR > cmdR_prev-100 && cmdR < cmdR_prev+100)) {
        #ifdef INVERT_R_DIRECTION
          pwmr = cmdR;
        #else
          pwmr = -cmdR;
        #endif
        #ifdef INVERT_L_DIRECTION
          pwml = -cmdL;
        #else
          pwml = cmdL;
        #endif
      }
    #endif

    // ####### SIDEBOARDS HANDLING #######
    #if defined(SIDEBOARD_SERIAL_USART2) && defined(FEEDBACK_SERIAL_USART2)
      sideboardLeds(&sideboard_leds_L);
      sideboardSensors((uint8_t)Sideboard_L.sensors);
    #endif
    #if defined(SIDEBOARD_SERIAL_USART3) && defined(FEEDBACK_SERIAL_USART3)
      sideboardLeds(&sideboard_leds_R);
      sideboardSensors((uint8_t)Sideboard_R.sensors);
    #endif
		
		// ############### PID CONTROL ################################
    #if defined PID_CONTROL
		      // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
      if (enable == 0 ) {    
        beepShort(6);                     // make 2 beeps indicating the motor enable
        beepShort(4); HAL_Delay(100);
        speed1Fixdt = speed2Fixdt = 0;      // reset filters
        enable = 1;                       // enable motors
        #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
        printf("-- Motors enabled --\r\n");
        #endif
      }
		//Motor_Pos( &MotorPosL, &MotorPosR); // Calculate motor electrical positions relative to startup positions
		// ####### LOW-PASS FILTER #######
      rateLimiter16(input1[inIdx].cmd , RATE, &speed1RateFixdt);
      rateLimiter16(input2[inIdx].cmd , RATE, &speed2RateFixdt);
      filtLowPass32(speed1RateFixdt >> 4, FILTER, &speed1Fixdt);
      filtLowPass32(speed2RateFixdt >> 4, FILTER, &speed2Fixdt);
		
      speed1 = (int16_t)(speed1Fixdt >> 16);  // convert fixed-point to integer
      speed2 = (int16_t)(speed2Fixdt >> 16);  // convert fixed-point to integer
		// uncomment for step input testing 
			//speed1 = ( main_loop_counter > 4)*500; 
			//speed2 = ( main_loop_counter >  4)*500; 
		// end step input testing	
			
		PIDL.input = speed1;//-input1[inIdx].cmd; scaled to +_1000 counts input
		PIDR.input = speed2;// input2[inIdx].cmd scaled to +- 1000 counts input
			
		#ifdef INVERT_R_DIRECTION
      PIDL.input = -PIDL.input1;       
    #endif
    #ifdef INVERT_L_DIRECTION
      PIDL.input1 = -PIDR.input1;
    #endif       
			
		// changed scale of rotation to mechanical degrees, 90 steps is a full rotation
		//	PIDL.feedback = (MotorPosL*2000)/5400; // scale to 2000 units per rotation   sf = .37 =2000/(360 deg*15pole pairs= 5400 elec deg)
		//	PIDR.feedback = (MotorPosR*2000)/5400;  //minimum step is 60 deg elec phase angle, or 4 deg mechanical angle
		PIDL.feedback = (MotorPosL*2000)/5400; // scale to 2000 units per rotation   sf = .37 =2000/(360 deg*15pole pairs= 5400 elec deg)
		PIDR.feedback = (MotorPosR*2000)/5400;  //minimum step is 60 deg elec phase angle, or 4 deg mechanical angle
		PID(&PIDL);// left pid control
	  //print_PID(PIDL);  
		PID(&PIDR);// right pid control
		//print_PID(PIDR);
		//limit pwm to 100 during first five seconds to keep turn on transients safe
		if(main_loop_counter < 1000){
		cmdL = CLAMP(PIDL.output,-100,100);
		cmdR = CLAMP(PIDR.output,-100,100);
		}
    else {	
		cmdL = CLAMP(PIDL.output,-1000,1000);
		cmdR = CLAMP(PIDR.output,-1000,1000);	
		}	
		pwml = cmdL;
		pwmr = cmdR;
		//if (main_loop_counter >= 2) pwml = pwmr=500;//speed test
		
		
		
		
     #endif
    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### DEBUG SERIAL OUT #######
    #if defined(DEBUG_SERIAL_USART2) || defined(DEBUG_SERIAL_USART3)
		   { 
		loop_time= HAL_GetTick()-tick_last;
				tick_last = HAL_GetTick();
//      if (main_loop_counter % 25 == 0) {    // Send data periodically every 125 ms
//        printf("in1:%i in2:%i cmdL:%i cmdR:%i \r\n",		
//          input1[inIdx].raw,        // 1: INPUT1
//          input2[inIdx].raw,        // 2: INPUT2
//          cmdL,                     // 3: output command: [-1000, 1000]
//				  cmdR );                   // 4: output command: [-1000, 1000]
//				}
//				if (main_loop_counter % 260 == 0) {    // Send data periodically every 1.3 ms	
//				printf("BADC:%i BV:%i TADC:%i T:%i\r\n",
//          adc_buffer.batt1,         // 5: for battery voltage calibration
//          batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC, // 6: for verifying battery voltage calibration
//          board_temp_adcFilt,       // 7: for board temperature calibration
//          board_temp_deg_c);        // 8: for verifying board temperature calibration
//				}
			/*	 if (main_loop_counter % 1 == 0) {    //  Send PID data periodically every 5 ms
        
				printf("IL %i FL %i IR %i FR %i\r\n",
				  PIDL.input,
					PIDL.feedback,
				  PIDR.input,
					PIDR.feedback
					 );}*/
					
          //cmdL,                     // 3: output command: [-1000, 1000]
          //cmdR,                     // 4: output command: [-1000, 1000]           
          //motAngleRight,
				  //MotorPosL,
          //MotorPosR);
					
				  
				  
					
				 
				
				//			printf("loopms %i 48 total xxxxxxxxxxxxxxxxxxxxxxx\r\n",loop_time);
//				printf("loop_time :%i motorAngleR:%i MotorPosR:%i pwmL:%i motorAngleL:%i MotorPosL:%i \r\n",
//          //rtY_Right.n_mot,
//				  loop_time,
//	  			motAngleRight,
//	  			MotorPosR,        // 1: INPUT1
////          input2[inIdx].raw,        // 2: INPUT2
//          pwml,                     // 3: output command: [-1000, 1000]
////          cmdR,                     // 4: output command: [-1000, 1000]           
////          //adc_buffer.batt1,         // 5: for battery voltage calibration
////          //batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC, // 6: for verifying battery voltage calibration
////          //board_temp_adcFilt,       // 7: for board temperature calibration
////          //board_temp_deg_c);        // 8: for verifying board temperature calibration
//					//rtY_Left.n_mot,
//					motAngleLeft,
//				  MotorPosL);
////          MotorPosR);
//				 
				
			}
    #endif

    // ####### FEEDBACK SERIAL OUT #######
 /*   #if defined(FEEDBACK_SERIAL_USART2) || defined(FEEDBACK_SERIAL_USART3)  // disabled for passthru
      if (main_loop_counter % 2 == 0) {    // Send data periodically every 10 ms
        Feedback.start	        = (uint16_t)SERIAL_START_FRAME;
        Feedback.cmd1           = (int16_t)input1[inIdx].cmd;
        Feedback.cmd2           = (int16_t)input2[inIdx].cmd;
        Feedback.speedR_meas	  = (int16_t)rtY_Right.n_mot;
        Feedback.speedL_meas	  = (int16_t)rtY_Left.n_mot;
        Feedback.batVoltage	    = (int16_t)(batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC);
        Feedback.boardTemp	    = (int16_t)board_temp_deg_c;

        #if defined(FEEDBACK_SERIAL_USART2)
          if(__HAL_DMA_GET_COUNTER(huart2.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_L;
            Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
        #if defined(FEEDBACK_SERIAL_USART3)
          if(__HAL_DMA_GET_COUNTER(huart3.hdmatx) == 0) {
            Feedback.cmdLed     = (uint16_t)sideboard_leds_R;
            Feedback.checksum   = (uint16_t)(Feedback.start ^ Feedback.cmd1 ^ Feedback.cmd2 ^ Feedback.speedR_meas ^ Feedback.speedL_meas 
                                           ^ Feedback.batVoltage ^ Feedback.boardTemp ^ Feedback.cmdLed);

            HAL_UART_Transmit_DMA(&huart3, (uint8_t *)&Feedback, sizeof(Feedback));
          }
        #endif
      }
    #endif*/

    // ####### POWEROFF BY POWER-BUTTON #######
    poweroffPressCheck();

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if ((TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20) || (batVoltage < BAT_DEAD && speedAvgAbs < 20)) {  // poweroff before mainboard burns OR low bat 3
      poweroff();
    } else if (rtY_Left.z_errCode || rtY_Right.z_errCode) {                                           // 1 beep (low pitch): Motor error, disable motors
      enable = 0;
      beepCount(1, 24, 1);
    } else if (timeoutFlgADC) {                                                                       // 2 beeps (low pitch): ADC timeout
      beepCount(2, 24, 1);
    } else if (timeoutFlgSerial) {                                                                    // 3 beeps (low pitch): Serial timeout
      beepCount(3, 24, 1);
    } else if (timeoutFlgGen) {                                                                       // 4 beeps (low pitch): General timeout (PPM, PWM, Nunchuk)
      beepCount(4, 24, 1);
    } else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING) {                             // 5 beeps (low pitch): Mainboard temperature warning
      beepCount(5, 24, 1);
    } else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1) {                                            // 1 beep fast (medium pitch): Low bat 1
      beepCount(0, 10, 6);
    } else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2) {                                            // 1 beep slow (medium pitch): Low bat 2
      beepCount(0, 10, 30);
    } else if (BEEPS_BACKWARD && ((speed < -50 && speedAvg < 0) || MultipleTapBrake.b_multipleTap)) { // 1 beep fast (high pitch): Backward spinning motors
      beepCount(0, 5, 1);
      backwardDrive = 1;
    } else {  // do not beep
      beepCount(0, 0, 0);
      backwardDrive = 0;
    }


    // ####### INACTIVITY TIMEOUT #######
    if (abs(cmdL) > 50 || abs(cmdR) > 50) {
      inactivity_timeout_counter = 0;
    } else {
      inactivity_timeout_counter++;
    }
    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1)) {  // rest of main loop needs maybe 1ms
      poweroff();
    }

    // HAL_GPIO_TogglePin(LED_PORT, LED_PIN);                 // This is to measure the main() loop duration with an oscilloscope connected to LED_PIN
    // Update main loop states
    cmdL_prev = cmdL;
    cmdR_prev = cmdR;
    main_loop_counter++;
  }
}
	
}

// ===========================================================
/** System Clock Configuration
*/
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL16;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /**Initializes the CPU, AHB and APB busses clocks
    */
  RCC_ClkInitStruct.ClockType           = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource        = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider       = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider      = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider      = RCC_HCLK_DIV1;

  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);

  PeriphClkInit.PeriphClockSelection    = RCC_PERIPHCLK_ADC;
  // PeriphClkInit.AdcClockSelection    = RCC_ADCPCLK2_DIV8;  // 8 MHz
  PeriphClkInit.AdcClockSelection       = RCC_ADCPCLK2_DIV4;  // 16 MHz
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  /**Configure the Systick interrupt time
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}
