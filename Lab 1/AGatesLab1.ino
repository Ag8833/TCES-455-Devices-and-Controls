/*
 * TCES 455 - Lab 1
 * Andrew Gates
 * This lab controls the heat through a resistor by not allowing it to 
 * get greater than or less than 5 degrees Celsius of room temperature.
 */


int ledPin = 13;
int Resistor = 3;

void setup()
{
  pinMode(ledPin, OUTPUT);
  pinMode(Resistor, OUTPUT);

  // initialize timer1
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 125;            // compare match register 16MHz/256/1khz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS10);    // 1024 prescaler 001
  TCCR1B |= (1 << CS11);    // 256 prescaler  100
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts();             // enable all interrupts
  Serial.begin(9600);
}

// Timer compare interrupt service routine to control the signal to the resistor
ISR(TIMER1_COMPA_vect)
{
  int temp = analogRead(A0);
  if (temp < 500) 
  {
    digitalWrite(ledPin, HIGH);
    digitalWrite(Resistor, HIGH);
  }
  else 
  {
    digitalWrite(ledPin, LOW);
    digitalWrite(Resistor, LOW);
  }
  Serial.println(temp);
}

void loop()
{

}



