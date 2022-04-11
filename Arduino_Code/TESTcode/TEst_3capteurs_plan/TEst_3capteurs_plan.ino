#include <AFMotor.h>
#include <Servo.h> // include Servo library 

int servoh = 180;   // 90;     // stand horizontal servo
int servov = 90;    //   90;     // stand vertical servo

//int servohLimitHigh = 180;
//int servohLimitLow = 65;

float stepd; 

int servovLimitHigh = 360;
int servovLimitLow = 0;

// LDR pin connections
//  name  = analogpin;
int ldrlt = 2; //LDR top left - BOTTOM LEFT    <--- BDG
int ldrrt = 3; //LDR top rigt - BOTTOM RIGHT 
int ldrld = 4; //LDR down left - TOP LEFT
int ldrrd = 3; //ldr down rigt - TOP RIGHT

// Connect a stepper motor with 200 steps per revolution (1.8 degree)
// to motor port #2 (M3 and M4)
AF_Stepper motor(200, 1);
 // Serial.println("Interleave coil steps");
float h_step = 0.9;

void setup() {
  Serial.begin(9600);
  delay(1000);
  motor.setSpeed(10);  // 10 rpm  == 60 deg/s 
}

void loop() {
  // put your main code here, to run repeatedly:
  int lt = analogRead(ldrlt); // top left
  int rt = analogRead(ldrrt); // top right
  int ld = analogRead(ldrld); // down left
  int rd = analogRead(ldrrd); // down rigt
  
  // int dtime = analogRead(4)/20; // read potentiometers  
  // int tol = analogRead(5)/4;
  int tol = 3;
  
  int avt = (lt + rt) / 2; // average value top
  int avd = (ld + rd) / 2; // average value down
  int avl = (lt + ld) / 2; // average value left
  int avr = (rt + rd) / 2; // average value right

  int dvert = avt - avd; // check the diffirence of up and down
  int dhoriz = avl - avr;// check the diffirence og left and rigt
  
  
//  Serial.print(avt);
//  Serial.print(" ");
//  Serial.print(avd);
//  Serial.print(" ");
//  Serial.print(avl);
//  Serial.print(" ");
//  Serial.print(avr);
//  Serial.print("   ");
//  Serial.print(dtime);
//  Serial.print("   ");
//  Serial.print(tol);
//  Serial.println(" ");
//  
    
if (-1*tol > dvert || dvert > tol) // check if the diffirence is in the tolerance else change vertical angle
  {
  if (avt > avd)
  {
    servov = ++servov;
     if (servov > servovLimitHigh) 
     { 
      servov = servovLimitHigh;
     }
  }
  else if (avt < avd)
  {
    servov= --servov;
    if (servov < servovLimitLow)
  {
    servov = servovLimitLow;
  }
  }
  
//  vertical.write(servov);
 }
  Serial.print(dhoriz);
  if (-1*tol > dhoriz || dhoriz > tol) // check if the diffirence is in the tolerance else change horizontal angle
  {
  if (avl > avr)
  {
    servoh = --servoh;
    if (servoh < servohLimitLow)
    {
    servoh = servohLimitLow;
    }
  }
  else if (avl < avr)
  {
    servoh = ++servoh;
     if (servoh > servohLimitHigh)
     {
     servoh = servohLimitHigh;
     }
  }
  else if (avl = avr)
  {
    // nothing
  }
  Serial.println("Angolo che sta dando in pasto al mototore verticale: ");
  
  Serial.print(" ");
  stepd = servoh/h_step ; 
  motor.step(stepd, FORWARD, INTERLEAVE); 
  Serial.print(stepd);
  }
  
}
