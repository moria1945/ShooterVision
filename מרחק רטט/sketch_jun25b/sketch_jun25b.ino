#define IR_PIN 27
void setup() {
  pinMode(IR_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int obstacleDetected = digitalRead(IR_PIN);
  if (obstacleDetected == LOW) {  // בחלק מהחיישנים LOW = מכשול
    Serial.println("מכשול זוהה!");
  } else {
    Serial.println("אין מכשול");
  }
  delay(200);
}
