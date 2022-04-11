///////////////////////////////////////////////////////////////////////////////
///////      TP de systeme d'entrainement - Heliostat arduino           ///////
///////         George Claudiu Andrei - Gabriel Gattaux                 ///////
/////// MIX5A                                                19/01/2021 ///////
///////////////////////////////////////////////////////////////////////////////
// the position servo initialization is mirror on the deck

#include <AFMotor.h>
#include <Servo.h>

// ####################################################################################################################################################

Servo myServo;

//Initial position for calculing
int servoh = 0;   // 90;     // stand horizontal servo
int servov = 0;    //   90;     // stand vertical servo

//Limit in angle of the altitude's motor
int servovLimitHigh = 100; //Maximum of the system (100 with secure of 23 deg)
int servovLimitLow = 0;

//Define desired steps - chnge each dtime in delay (ms)
float stepd_alt;
int dtime = 20;

// LDR pin connections
// Triangle Graph TOP is the side perpendicular to the mirror when position mirror on the deck 
int ldrlt = A4; //LDR top
int ldrld = A3; //LDR down left 
int ldrrd = A5; //ldr down rigt 
int inite =0;
// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
AF_Stepper motor_az(200, 2);

//
void setup()
{
  myServo.attach(9);
  Serial.begin(9600);
  Serial.println("Test motor upper");
  delay(1000);
  
  //Settings speed of the stepper motors, 
  motor_alt.setSpeed(10);  // 10 rpm  == 60 deg/s 
  
  delay(2000);
  
  motor_alt.step(50, BACKWARD, DOUBLE); 
  myServo.write(130);

  //motor_alt.step(50, FORWARD, DOUBLE); 
}

void loop() 
{
  inite = ++inite;
  // BE EXTREMELY CAREFUL WITH THE SENS OF ROTATION, AT THE BEGINING 
  //motor_alt.step(0, FORWARD, DOUBLE); 
 
   delay(dtime);

}
