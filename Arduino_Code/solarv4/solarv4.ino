///////////////////////////////////////////////////////////////////////////////
///////      TP de systeme d'entrainement - Heliostat arduino           ///////
///////         George Claudiu Andrei - Gabriel Gattaux                 ///////
/////// MIX5A                                                19/01/2021 ///////
///////////////////////////////////////////////////////////////////////////////
// the position servo initialization is mirror on the deck

#include <AFMotor.h>
#include <Servo.h>
// --NOTE-- ###################################################################################################################################################################################
//https://arduinogetstarted.com/tutorials/arduino-light-sensor
//https://wiki.seeedstudio.com/Grove-Luminance_Sensor/
//https://wiki.seeedstudio.com/Grove-Light_Sensor/          Difference between light sensor and luminance sensor
//
//  - TO DO -
// -------- MECA ------ OK 
// --> Hole of 4mm diameter in the center of the mirror(PLA)              OK
// --> Hole of diameter of the 3D insert, in the last 3D print triangle   OK
// --> Put insert inside with soldering tools                             OK
// --> Mount chassis and triangle with screw                              OK
// --> Change the chassis with the new one, keep the base and the coupler OK
// --> Fix arduino and stuff                                              OK
//
// -------- ARDUINO --- 
// --> Create a test function for the direction of rotation motor altitude, to check value of FORWARD OR BACKWARD (Control part, below)
// --> Test this code for azimuth (1 priority or after MECA) and altitude 
// --> (Maybe this aint gonna work so try again) - Looking at the tolerance
//
// -------- DELIVERY --
// --> Do the video 
// --> Report
// --> Folder
// --> Presentation
//
// -------- OTHERS ----
// --> We need to find LDR sensors like the 2 others, or 1 more
// --> And battery 12V 2A (security)
// --> Looking curve Couple vs Speed of Rotation 
// --> Adding the Time library ? 
//
// ####################################################################################################################################################

//Initial position for calculing
int servoh = 0;   
int servov = 90;    //   180;     // stand normal to the horizon mirror



//Limit in angle of the altitude's motor
int servovLimitHigh = 180; //Maximum of the system
int servovLimitLow = 180-120; //10 for safety; 125 for the dead zone according to the initiale position

//Define desired steps - chnge each dtime in delay (ms)
float stepd_az;
float stepd_alt;
int dtime = 20;
float step_az = 1.8;
int inite = 0; 

// LDR pin connections
// Triangle Graph TOP is the side perpendicular to the mirror when position mirror on the deck 
int ldrlt = A4; //LDR top
int ldrld = A3; //LDR down left 
int ldrrd = A5; //ldr down rigt 

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
AF_Stepper motor_az(200, 2);
Servo myServo;

// They're used here to set pin numbers:
const int BUTTON_PIN = A0;  // the number of the pushbutton pin
const int LED_PIN =  A1;   // the number of the LED pin

int buttonState;

// SPEED MOTOR AZ
float deg_sec_az = 60; //Speed of the motor azimuth desired in deg/s (Looking at the curve of Couple/Speed of the motor )
float rpm_az = deg_sec_az / 6; //[rpm]
float rad_sec_az = deg_sec_az * PI/180; //[rad/s]
// ---------------------------------------------

int tol = 50;
  
void setup()
{
  Serial.begin(9600);
  Serial.print("Double coil stepper motor_az");
  Serial.print("Servo motor_alt");
  
  delay(1000);
  
  //Settings speed of the stepper motor and attach the servo 
  motor_az.setSpeed(rpm_az);  // 10 rpm  == 60 deg/s 
  myServo.attach(9);

  myServo.write(90);

  
  pinMode(LED_PIN, OUTPUT);
  // initialize the pushbutton pin as an pull-up input:
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() 
{
  // values of the ldr
  int lt = analogRead(ldrlt); // top 
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt
  
  // int dtime = analogRead(4)/20; // read potentiometers  
  // int tol = analogRead(5)/4;
  

  //Changed to fit wth 3 sns in triangle see schema
  int avt = lt;            // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (ld + lt) / 2; // average value left
  int avr = (rd + lt) / 2; // average value right

  int dvert = avt - avd; // check the diffirence of up and down
  int dhoriz = avl - avr;// check the diffirence og left and rigt
  
  
  Serial.print(lt);
  Serial.print(" ");
  Serial.print(ld);
  Serial.print(" ");
  Serial.print(rd);
  Serial.print(" ");
  Serial.print(avr);
  Serial.print("   ");
  Serial.print(dtime);
  Serial.print("   ");
  Serial.print(tol);
  Serial.println(" ");
  Serial.print(dvert);
  Serial.println(" ");
  Serial.print(dhoriz);
  Serial.println(" ");

  
  
  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN);
  
  // Set the system workings control ON according to the state of button (and LED)
  if(buttonState == 0)
  {
    inite = 1;
  }
  
  // If button is pressing, Control 
  if(inite == 1) 
  {         
    analogWrite(LED_PIN, 255); // turn on LED
    if (-1*tol > dvert || dvert > tol) // check if the diffirence is in the tolerance else change vertical angle
    {
      if (avt > avd)
      { 
        servov = --servov ;
        if (servov < servovLimitLow) 
        { 
          servov = servovLimitLow;
        }
      }
      else if (avt < avd)
      {
        servov= ++servov;
        if (servov > servovLimitHigh)
        {
          servov = servovLimitHigh;
        }
  }
  Serial.print("altitude angle : ");
  Serial.print(servov);
  Serial.println(" ");
  myServo.write(servov);
  }
  
  if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
  {
  if (avl > avr)
  {
    servoh = servoh - step_az;
  }
  else if (avl < avr)
  {
    servoh = servoh + step_az;
  }
  else if (avl = avr)
  {
    // nothing
  }
  Serial.println("Azimuth anglee :  ");
  Serial.print(servoh);
  stepd_az = servoh / step_az ; 
  Serial.print("Step desired azimuth : ");
  Serial.print(stepd_az);

  if (stepd_az < 0)
  {
    motor_az.step(abs(stepd_az), BACKWARD, DOUBLE);
    stepd_az = 0;
  }
  else if (stepd_az > 0)
  {
    motor_az.step(abs(stepd_az), FORWARD, DOUBLE);
    stepd_az = 0;
  }   //Control the motor using interleave = less couple
    }
  }
  
  else                         // otherwise, button is not pressing
   analogWrite(LED_PIN, 0);  // turn off LED

   delay(dtime);

}
