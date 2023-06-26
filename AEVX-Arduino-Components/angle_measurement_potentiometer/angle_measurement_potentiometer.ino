const int potPin = A3;  // Analog input pin that the potentiometer is attached to
const int potDegrees = 300;  // For a standard potentiometer with 270 degree range.
const float referenceVoltage = 5.0;  // The reference voltage for Arduino ADC. Change this if your Arduino uses a different reference voltage.
float voltage = 0;  // The voltage across the wiper terminal
int potValue = 0;  // The raw value read from the pot
int angle = 0;  // The calculated rotation angle

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
}

void loop() {
  // read the analog in value:
  potValue = analogRead(potPin);

  // calculate the voltage across the wiper terminal:
  voltage = (potValue / 1023.0) * referenceVoltage;

  // map the pot value to the range of the pot's rotation (in degrees)
  angle = map(potValue, 0, 1023, 0, potDegrees);

  // print the results to the Serial Monitor:
  //Serial.print("potentiometer = ");
  //Serial.print(potValue);
  //Serial.print("\t voltage = ");
  //Serial.print(voltage);
  //Serial.print("\t angle = ");
  Serial.println(angle);

  // wait 10 milliseconds before the next loop
  // for the analog-to-digital converter to settle
  // after the last reading:
  delay(10);
}
