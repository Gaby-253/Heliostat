float testmodulo;
float i = 0;

void setup() {
  Serial.begin(9600);
  // put your setup code here, to run once:
  
  
}

void loop() {
  
  i = i + 0.2;
  testmodulo = i%1.8;
  // put your main code here, to run repeatedly:
  Serial.print("Step desired azimuth : ");
  Serial.print(testmodulo);
  delay(1000)
}
