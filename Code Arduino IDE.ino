#include <Servo.h>

Servo servoX;
Servo servoY;

// Set these pins according to your connections
const int servoXPin = 9;
const int servoYPin = 10;

// Current servo angles
int currentAngleX = 90; // Starting neutral position
int currentAngleY = 90; // Starting neutral position

void setup() {
  servoX.attach(servoXPin);
  servoY.attach(servoYPin);

  // Initialize servos to their neutral position (or a safe starting point)
  // This will move them to 90 degrees when the Arduino starts or resets.
  servoX.write(currentAngleX);
  servoY.write(currentAngleY);

  Serial.begin(9600); // Start serial communication at 9600 baud
  Serial.println("Arduino ready for servo commands.");
}

void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n'); // Read the incoming string until newline
    // Expected format: "X_ANGLE,Y_ANGLE" e.g., "90,90"

    int commaIndex = data.indexOf(',');
    if (commaIndex != -1) {
      String xAngleStr = data.substring(0, commaIndex);
      String yAngleStr = data.substring(commaIndex + 1);

      int desiredAngleX = xAngleStr.toInt();
      int desiredAngleY = yAngleStr.toInt();

      // Constrain angles to the valid servo range (0-180 degrees)
      desiredAngleX = constrain(desiredAngleX, 0, 180);
      desiredAngleY = constrain(desiredAngleY, 0, 180);

      // Move servos only if the angle has changed to minimize jitter
      if (desiredAngleX != currentAngleX) {
        servoX.write(desiredAngleX);
        currentAngleX = desiredAngleX;
      }
      if (desiredAngleY != currentAngleY) {
        servoY.write(desiredAngleY);
        currentAngleY = desiredAngleY;
      }

      // Optional: Print received data for debugging on Arduino Serial Monitor
      // Serial.print("Received X: ");
      // Serial.print(desiredAngleX);
      // Serial.print(", Y: ");
      // Serial.println(desiredAngleY);
    }
  }
}