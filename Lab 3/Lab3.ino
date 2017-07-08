/*
   TCES 455 - Lab 3
   Andrew Gates
   This lab controls the water level in a tank to maintain a preset
   equilibrium level. This code does not use any controllers.
*/
int pumpPin = 13;
int Resistor = 3;
int fulidRisitance = 0;
int button = 5;
const int SIZE = 20;
double myArray[SIZE][SIZE];

void setup()
{
  pinMode(pumpPin, OUTPUT);
  pinMode(Resistor, OUTPUT);
  pinMode(button, INPUT_PULLUP);

  Serial.begin(9600);
  holdTheDoor();
}

//Function to maintain equilibrium
void holdTheDoor()
{
  fillToTop();
  int pumpSpeed = 200;
  analogWrite(pumpPin, pumpSpeed);
  while (1) 
  {
    int diff = analogRead(A0) - 430;

    //If water level is below desired level
    if (diff > 0)
    {
      pumpSpeed += diff;
      if (pumpSpeed > 255) pumpSpeed = 255;
      analogWrite(pumpPin, pumpSpeed);
      Serial.print("pumpSpeed^^^^^: "); Serial.print(pumpSpeed);
      Serial.print("   Diff "); Serial.println(diff);
    }

    //If water level is above desired level
    else
    {
      pumpSpeed -= diff;
      if (pumpSpeed < 0) pumpSpeed = 0;
      analogWrite(pumpPin, pumpSpeed);
      Serial.print("pumpSpeed|||||: "); Serial.print(pumpSpeed);
      Serial.print("   Diff "); Serial.println(diff);
    }
  }
}

//Function to fill our water tank to the top to allow for calculations
void fillToTop()
{
  //-------------PUMP 2----------------------
  double curr = millis();
  while (analogRead(A0) > 435)
  {
    analogWrite(pumpPin, 255);
    Serial.print("pumping2  "); Serial.println(analogRead(A0));
  }
  analogWrite(pumpPin, 0);
  double finish = millis() - curr;
}

void loop()
{
  int temp = analogRead(A0);
  Serial.println(temp);
}
