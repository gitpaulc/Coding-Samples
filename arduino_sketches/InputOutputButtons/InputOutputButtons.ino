
// By Paul Cernea

const int ledPin = 5;
const int buttonOn = 9;
const int buttonOff = 8;

uint8_t buttonState = LOW; // Off, 0.
void setup() 
{
  pinMode(ledPin, OUTPUT);
  pinMode(buttonOn, INPUT_PULLUP);  
  pinMode(buttonOff, INPUT_PULLUP);
  buttonState = LOW;
  digitalWrite(ledPin, LOW);
}

void loop() 
{
  if (digitalRead(buttonOn) == LOW)
  {
    if (buttonState == HIGH) { return; }
    buttonState = HIGH;
  }
  else if (digitalRead(buttonOff) == LOW)
  {
    if (buttonState == LOW) { return; }
    buttonState = LOW;
  }
  else { return; }
  digitalWrite(ledPin, buttonState);
}
