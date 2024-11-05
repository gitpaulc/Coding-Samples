#include <Servo.h>
// By Paul Cernea.

Servo servo;
const int degrees = 1; // Multiplier.
int delayMilliseconds = 500;
int dt = -10;

void setup()
{
  servo.attach(9);
  servo.write(90 * degrees);
  delayMilliseconds = 500;
}
void pause() { delay(delayMilliseconds); }

void loop()
{
  servo.write(90 * degrees); pause();
  delayMilliseconds += dt;
  servo.write(30 * degrees); pause();
  delayMilliseconds += dt;
  servo.write(90 * degrees); pause();
  delayMilliseconds += dt;
  servo.write(150 * degrees); pause();
  delayMilliseconds += dt;
  if (delayMilliseconds <= 250) { dt = 10; }
  if (delayMilliseconds >= 500) { dt = -10; }
}