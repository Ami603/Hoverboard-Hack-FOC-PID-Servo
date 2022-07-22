
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
uint8_t  ppm_count = 0;
uint8_t  pwm_count = 0;
uint32_t timeoutCntGen = TIMEOUT;
uint8_t  timeoutFlgGen = 0;
uint8_t  nunchuk_data[6] = {0};

uint8_t i2cBuffer[2];

extern I2C_HandleTypeDef hi2c2;
extern DMA_HandleTypeDef hdma_i2c2_rx;
extern DMA_HandleTypeDef hdma_i2c2_tx;

extern int motAngleLeft;
extern int motAngleRight;
extern int MotorPosL ;
extern int MotorPosR ;

void Motor_Pos(){
static int motAngleLeftLast = 0;
static int motAngleRightLast = 0;	
static int cycleDegsL = 0 ;  //wheel encoder roll over count in deg
static int cycleDegsR = 0 ;  //wheel encoder roll over count in deg	
int diffL = 0;
int diffR = 0;
static int cnt = 0;	
	if(cnt == 0) 
	{
		motAngleLeftLast = motAngleLeft ;
		motAngleRightLast = motAngleRight ;
	}
	diffL = motAngleLeft - motAngleLeftLast ;
	//if(cnt % 25 == 0 ) {printf( " diffL :%i, diffR :%i , motAngleRightLast :%i ",diffL , diffR, motAngleRightLast );}
	
if( diffL < -180 ) { cycleDegsL = cycleDegsL + 360; }
else 
if(diffL > 180) { cycleDegsL = cycleDegsL  - 360;}
MotorPosL = motAngleLeft + cycleDegsL;

	diffR = motAngleRight - motAngleRightLast ;
if( diffR < -180 ) { cycleDegsR = cycleDegsR + 360; }
else 
if(diffR > 180) { cycleDegsR = cycleDegsR  - 360;}
MotorPosR = motAngleRight + cycleDegsR;
motAngleLeftLast = motAngleLeft;
motAngleRightLast = motAngleRight;
cnt++;

}

void PID( PID_DATA *p){
	
	/* typedef struct 
     {
		 float Kp;
		 float Ki;
		 int dz ;
		 int limit;
		 int error;
		 int cum_error;
		 int input;
	   int feedback;
		 int output;
		 
		 }PID_DATA ;
   */
	p->error = p->input -  p->feedback;
	p->cum_error = p->cum_error + p->error;
	if (p->cum_error > p->limit ) {p->cum_error = p->limit;}
	else if (p->cum_error< -p->limit) {p->cum_error = - p->limit ;}
	p->output = p->error*p->Kp + p->cum_error*p->Ki ;
	
	
}
void print_PID(PID_DATA p){
	static int cnt = 0 ;
	if(cnt == 0) {
		printf("PID INPUT PARAMETERS p.Kp:%f p.Ki:%f p.dz:%i p.limit:%i \r\n",
          p.Kp,        // PID Kp gain  
          p.Ki,        // 2: PID Ki gain
          p.dz,        // PID error dead zone         
          p.limit);     // PID integrator limit
		
		
		
	}
		
	if(cnt % 25 == 0){
	
				printf("p.error:%i p.cum_error:%i p.input:%i p.feedback:%i p.output%i \r\n",
          
          p.error,     // pid error           
          p.cum_error, // pid integral 
					p.input  ,   // input [-1000, 10]00]
					p.feedback,  // feedback [-1000,1000]
				  p.output);   // output [-1000, 1000]
	}
	cnt++ ;
}

