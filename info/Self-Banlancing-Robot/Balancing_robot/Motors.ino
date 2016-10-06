// Motor Pins
int enablea = 3;
int enableb = 9;
int a1 = 5;
int a2 = 4;
int b1 = 10;
int b2 = 11;

// Prueba set point
double distancia = 0;

void InitMotors() {

  // Set pins as outputs
  pinMode(enablea, OUTPUT);
  pinMode(enableb, OUTPUT);
  pinMode(a1, OUTPUT);
  pinMode(a2, OUTPUT);
  pinMode(b1, OUTPUT);
  pinMode(b2, OUTPUT);

  // Set direction to none direction
  digitalWrite(a1, HIGH);
  digitalWrite(a2, HIGH);
  digitalWrite(b1, HIGH);
  digitalWrite(b2, HIGH);

}


void MotorControl(double out) {

  // Sets direction
  if (out > 0) {             // forward
    digitalWrite(a1, HIGH);
    digitalWrite(a2, LOW);
    digitalWrite(b1, LOW);
    digitalWrite(b2, HIGH);
  } else {                    // backward
    digitalWrite(a1, LOW);
    digitalWrite(a2, HIGH);
    digitalWrite(b1, HIGH);
    digitalWrite(b2, LOW);
  }

  byte vel = abs(out);    // Absolute value of velocity

  // Checks velocity fits the max ouptut range
  if (vel < 0)
    vel = 0;
  if (vel > 255)
    vel = 255;

  // Writes the PWM
  analogWrite(enablea, vel);
  analogWrite(enableb, vel);

}
