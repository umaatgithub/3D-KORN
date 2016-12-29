/*//#include "WProgram.h"
#include <Servo.h>
#include <digitalWriteFast.h>  // library for high performance reads and writes by jrraines
                               // see http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1267553811/0
                               // and http://code.google.com/p/digitalwritefast/
 
// It turns out that the regular digitalRead() calls are too slow and bring the arduino down when
// I use them in the interrupt routines while the motor runs at full speed creating more than
// 40000 encoder ticks per second per motor.

// Encoder
#define c_RightEncoderInterrupt 3
#define c_RightEncoderPinA 2
#define c_RightEncoderPinB 4

volatile int val = 0;

void setup()
{
  Serial.begin(115200);

  // Encoder
  pinMode(c_RightEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  digitalWrite(c_RightEncoderPinA, LOW);  // turn on pullup resistors
  attachInterrupt(c_RightEncoderPinA, HandleRightMotorInterruptA, CHANGE);
}

void loop()
{
  val = digitalRead(c_RightEncoderPinA);   // read the input pin
  //   val2 = digitalRead(c_RightEncoderPinB);   // read the input pin

  //Serial.print("\n");
    Serial.print(val);
     // Serial.print("\n");
  delay(20);
}
 
 
// Interrupt service routines for the right motor's quadrature encoder
void HandleRightMotorInterruptA()
{
  val=2;
}

*/
/*
const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = 0;
byte degree_turned = 0;
byte old_state = 0;
byte count_risings = 0;

void setup() 
{
    Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), blink, CHANGE);
}

void loop() 
{
  digitalWrite(ledPin, state);
      Serial.print(state);
      delay(50);
      
  if (degree_turned == 1)
  {
      Serial.print("\n");
    Serial.print("give em hell");
      Serial.print("\n");
    degree_turned = 0;
  }
}

void blink() 
{
    state = digitalRead(interruptPin);   // read the input pin
    if (old_state == 0 && state == 1)
    {
      count_risings++;
    }
    
    old_state = state;
    
    if (state == 1)
    {
      if (count_risings == 12)  //that means 1 degree turned by the platform (i.e. 12 pulses received from the encoder)
      {
        degree_turned = 1;
        count_risings = 0;
      }
    }
    
  
}

*/


const byte ledPin = 13;
const byte interruptPin = 2;
volatile byte state = 0;
byte degree_turned = 0;
byte old_state = 0;
unsigned long int count_risings = 0;
int incomingCommand[5];   // for incoming serial data
int rotationsCommand = 0;   // for incoming serial data assuring number of rotations
bool data_available=false;
int rot_num=1;    //  1 rotation from default

void setup() 
{
    pinMode(LED_BUILTIN, OUTPUT);
    Serial.begin(4800);
  pinMode(ledPin, OUTPUT);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin), TDK_ISR_blink, RISING);
}

void loop() 
{

          if (Serial.available() > 0) 
          {
          incomingCommand[0] = Serial.read();
          data_available = true;
          }

          if (data_available = true)
          {
            data_available = false;
                if (incomingCommand[0] == 0) //stop platform
                  {
                    TDK_stopMotor;
                  }
                else if (incomingCommand[0] == 9)  //start platform
                  {
                    //PWMS TO THEIR INITIAL POWER
                    //AQUIRE CRUISE VELOCITY THROUGH PWM
                  }
                 else if (incomingCommand[0] < 9 && incomingCommand[0] > 0)   //set number of rotations
                 {
                  int rot_num = incomingCommand[0];
                 }

                 
          }
}



void TDK_ISR_blink()  //interruption service function for when encoder gives a pulse
{
    count_risings++;
      /*Uncomment the two following lines to print in the serial the number pf interruptions occured*/
      //Serial.print(count_risings);
      //Serial.print("\n");
      
      if ( count_risings == 60 )  //that means 1 degree turned by the platform (i.e. 12 pulses received from the encoder) -> 5 degrees are 60 pulses
      {
        degree_turned += 1;
        count_risings = 0;
        Serial.write(5);
      }

      if ( degree_turned == rot_num * 360 )
      {
        TDK_stopMotor();
      }      
}


void TDK_stopMotor()
{
  //SET PWMS TO ZERO SO PLATFORM STOPS
}











