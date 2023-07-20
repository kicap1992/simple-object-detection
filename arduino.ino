#include <Servo.h>

Servo servo1;
String command;

int position = 90;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);
  servo1.write(position);
}

void loop() {
  // degree();
   if (Serial.available()) {
    command = Serial.readStringUntil('\n');
    command.trim();
    if (command.equals("left")) {
      position = position - 2;
      servo1.write(position);
    } else if (command.equals("right")) {
      position = position + 2;
      servo1.write(position);
    } else if (command.equals("center")) {
      servo1.write(position);
    }
  }
}