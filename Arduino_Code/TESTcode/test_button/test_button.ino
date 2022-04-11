/*
 * Created by ArduinoGetStarted.com
 *
 * This example code is in the public domain
 *
 * Tutorial page: https://arduinogetstarted.com/tutorials/arduino-button-led
 */

// constants won't change. They're used here to set pin numbers:
const int BUTTON_PIN = A0;  // the number of the pushbutton pin
const int LED_PIN =  A1;   // the number of the LED pin

// variables will change:
int buttonState = 0;   // variable for reading the pushbutton status

void setup() {
  Serial.begin(9600);
  // initialize the LED pin as an output:
  pinMode(LED_PIN, OUTPUT);
  // initialize the pushbutton pin as an pull-up input:
  // the pull-up input pin will be HIGH when the switch is open and LOW when the switch is closed.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(BUTTON_PIN);

  // control LED according to the state of button
  if(buttonState == 0){         // If button is pressing
    analogWrite(LED_PIN, 255); // turn on LED
    delay(4000);}
  else                           // otherwise, button is not pressing
    analogWrite(LED_PIN, 0);  // turn off LED

  Serial.print(LED_PIN);
}
