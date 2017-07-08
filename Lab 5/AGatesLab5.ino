
#include <SoftwareSerial.h>
#include <PID_v1.h>

double sensorVal[] = {0, 0, 0};
double DIAM = 2;  //inches
int potPin = A1;
int pumpPin = 13;
int Resistor = 3;
int fulidRisitance = 0;
int caliButton = 3;
const int SIZE = 20;
double myArray[SIZE][SIZE];
int TOP;
double Area = 0;
double highSide;
double lowSide;
double level = 0;
int pumpSpeed = 0;

PID myPID(&sensorVal[0], &sensorVal[1], &level, 1.3, .5, 0, DIRECT);

void setup()
{
  myPID.SetMode(AUTOMATIC);
  //desired fequency is 10 Hz prescaler as 1024
  //[16,000,000 / (prescaler * 10) ] -1 = 1561.5
  //set timer1 interrupt at 1Hz
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  // set compare match register for 1hz increments
  OCR1A = 1562;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS10 and CS12 bits for 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);


  pinMode(pumpPin, OUTPUT);
  pinMode(Resistor, OUTPUT);
  pinMode(caliButton, INPUT_PULLUP);
  Serial.begin(115200);

  calibrateHeight();
  findArea();
  findR();
  // findPk();
  holdTheDoor();

  sei();//allow interrupts
}

ISR(TIMER1_COMPA_vect) {
  //generates pulse wave of frequency 10Hz = 0.1kHz
  Serial.print("Height: ");
  Serial.print(sensorVal[0]); //height
  Serial.print(", PumpSpeed: ");
  Serial.print(sensorVal[1]);   //pumpspeed
  Serial.print(", Diff: ");
  Serial.println(sensorVal[2]); //Difference
}

void calibrateHeight() {
  int pumpSpeed;
  Serial.println("Hit the button at 2inches ");
  while (digitalRead(caliButton) != LOW) {
    pumpSpeed = map(analogRead(potPin), 0, 1024, 0 , 255);
    analogWrite(pumpPin, pumpSpeed);
    delay(50);
  }
  lowSide = analogRead(A0);
  delay(500);
  Serial.println("Hit the button at 10inches ");
  while (digitalRead(caliButton) != LOW) {
    pumpSpeed = map(analogRead(potPin), 0, 1024, 0 , 255);
    analogWrite(pumpPin, pumpSpeed);
    delay(50);
  }
  highSide = analogRead(A0);
  delay(500);
  Serial.println("Boom! Calibrated!");
  TOP = highSide;
  pumpSpeed = 0;
  analogWrite(pumpPin, pumpSpeed);
}

void findArea() {
  Area = ((DIAM / 2.0) * 3.1416);
  Serial.print("Area is: ");
  Serial.println(Area);
}


void holdTheDoor() {
  
  fillTo(analogRead(potPin));
  pumpSpeed = 20;
  analogWrite(pumpPin, pumpSpeed);
  while (digitalRead(caliButton) != LOW) {
    int diff = analogRead(A0) - (TOP - 5);
    if (diff > 0) {
      //pumpSpeed += diff;
      if ((pumpSpeed + diff) > 255) pumpSpeed = 255;
      else pumpSpeed += diff;
      myPID.Compute();
      analogWrite(pumpPin, pumpSpeed);
      //      Serial.print("pumpSpeed^^^^^: "); Serial.print(pumpSpeed);
      //      Serial.print("   Diff "); Serial.println(diff);
    } else {
      if ((pumpSpeed + diff) < 0) pumpSpeed = 0;
      else pumpSpeed += diff;
      analogWrite(pumpPin, pumpSpeed);
      //      Serial.print("pumpSpeed|||||: "); Serial.print(pumpSpeed);
      //      Serial.print("   Diff "); Serial.println(diff);
    }
    sensorVal[0] = analogRead(A0);
    sensorVal[1] = pumpSpeed;
    sensorVal[2] = diff;
  }
}

void fillTo(double level) {
  int pumpSpeed;
  double currLevel;
  level = map(level, lowSide, highSide, 2.00, 10.00);
  pumpSpeed = map(analogRead(potPin), 0, 1024, 0 , 255);
  while (analogRead(A0) > level) {
    analogWrite(pumpPin, 255);
    currLevel = map(analogRead(A0), lowSide, highSide, 2.00, 10.00);
    Serial.print("Filling to...  ");Serial.print(level);
    Serial.print("  Currently at ");Serial.println(currLevel);
  }
  analogWrite(pumpPin, 0);
}

void findR() {
  fillTo(TOP);  //10inches hopefully
  double currH = map(analogRead(A0), lowSide, highSide, 2.00, 10.00);
  double initial = millis();
  double timeT;
  while (analogRead(A0) < 500) {
    timeT = millis() - initial;
    Serial.print("h: "); Serial.print(currH);
    Serial.print(", t: "); Serial.println(timeT);
    delay(750);
  }
  //  printArray()
  double fulidRisitance;
}

void loop()
{
  while (digitalRead(caliButton) != LOW) {
    int temp = analogRead(A0);
    Serial.print("IN LOOP!  Height: "); Serial.println(temp);
  }
}



