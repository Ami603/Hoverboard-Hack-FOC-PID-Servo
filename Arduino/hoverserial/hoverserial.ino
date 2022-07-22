// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
// 
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD   	// [-] Start frme definition for reliable serial communication
#define START_RETURN        0xCDAB
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      127       // [-] Maximum speed for testing
//#define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)

#include <SoftwareSerial.h>
SoftwareSerial HoverSerial(2,3);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;
SerialCommand NewCommand;
SerialCommand Cmd;
typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;

// ########################## SETUP ##########################
void setup() 
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");
  Serial1.begin(SERIAL_BAUD);
  //HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command Send(subir,inclinar);
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  Serial1.write((uint8_t *) &Command, sizeof(Command)); 
}

// ########################## RECEIVE ##########################
void Receive()
{
 
    //Check for new data availability in the Serial buffer
    if (Serial1.available()) {
        incomingByte 	  = Serial1.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        Serial.print(":");
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	 // Initialize if new data is detected
        p       = (byte *)&NewCommand;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;	
    } else if (idx >= 2 && idx < sizeof(SerialCommand)) {  // Save the new received data
        *p++    = incomingByte; 
        idx++;
    }	
    
    // Check if we reached the end of the package
    if (idx == sizeof(SerialCommand)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewCommand.start ^ NewCommand.steer ^ NewCommand.speed);

        // Check validity of the new data
        if (NewCommand.start == START_FRAME /*&& checksum == NewCommand.checksum*/) {
            // Copy the new data
            memcpy(&Cmd, &NewCommand, sizeof(SerialCommand));

            // Print data to built-in Serial
            Serial.print(" 1: ");   Serial.print(Cmd.steer);
            Serial.print(" 2: ");  Serial.println(Cmd.speed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

// ########################## LOOP ##########################

void loop()
{
  int8_t subiriz,subirder,incliz,inclder;
  int16_t subir,inclinar;
  for(int i=90; i<110;i++ )
  {
    subiriz = i; subirder = i;
    incliz = 100; inclder= 100;
    
    subir = (subiriz << 8) | subirder ;
    inclinar = (incliz <<8) | inclder ;
    Send(subir,inclinar);
    delay(10);
  }
    for(int i=110; i>90;i-- )
  {
    subiriz = i; subirder = i;
    incliz = 100; inclder=100;
    subir = (subiriz << 8) | subirder ;
    inclinar = (incliz <<8) | inclder ;
    Send(subir,inclinar);
    delay(10);
  } 
}

// ########################## END ##########################
