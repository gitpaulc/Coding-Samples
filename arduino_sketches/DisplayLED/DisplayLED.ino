
// By Paul Cernea

#define RED 6
#define GREEN 5
#define BLUE 3

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

void setup()
{
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);

  digitalWrite(RED, HIGH);
  digitalWrite(GREEN, LOW);
  digitalWrite(BLUE, LOW);

  redValue = 255;
}

void resetRgb()
{
  redValue = greenValue = blueValue = 0;
}

const int delayTime = 10; // milliseconds
void interpolate(uint8_t pin0, int& val0, uint8_t pin1, int& val1)
{
  const int interval = 1; // 1;
  val0 = 255;
  val1 = 0;
  for(int i = 0; i < 255; i += interval)
  {
    val0 -= interval;
    val1 += interval;
    analogWrite(pin0, val0);
    analogWrite(pin1, val1);
    delay(delayTime);
  }
}

void delayOnRed()
{
  analogWrite(RED, 255);
  delay(delayTime * 100);
}

void loop()
{
  resetRgb();
  delayOnRed(); // Nice effect, though unnecessary.
  interpolate(RED, redValue, GREEN, greenValue);
  interpolate(GREEN, greenValue, BLUE, blueValue);
  interpolate(BLUE, blueValue, RED, redValue);
}

