#define TRIG_PIN 5   // פין TRIG של החיישן מחובר ל-D5
#define ECHO_PIN 18  // פין ECHO של החיישן מחובר ל-D18

void setup() {
  Serial.begin(115200); // הפעלת תקשורת סריאלית
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration;
  float distance_cm;

  // שליחת פולס קצר לפין TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // קריאת הזמן שלוקח לפולס לחזור מהחיישן
  duration = pulseIn(ECHO_PIN, HIGH);

  // חישוב מרחק בס"מ (מהירות הקול: 0.034 ס"מ למיקרושנייה, לחלק ב־2 כי זה הלוך וחזור)
  distance_cm = duration * 0.034 / 2;

  // הדפסת המרחק לסריאל מוניטור
  Serial.print("מרחק: ");
  Serial.print(distance_cm);
  Serial.println(" ס\"מ");

  delay(500); // השהייה לפני הקריאה הבאה
}

