
// By Paul Cernea

const int secondsToWait = 5;

void setup()
{
  // OUTPUT == 1
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  // i == 0 is LOW (voltage level)
  // i == 1 is HIGH (voltage level)
  for (uint8_t i = 0; i < 2; ++i)
  {
    digitalWrite(LED_BUILTIN, i);
    delay(secondsToWait * 1000);
  }
}
