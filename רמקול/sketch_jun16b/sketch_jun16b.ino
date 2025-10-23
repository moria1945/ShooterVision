#define SPEAKER_PIN 25

void setup() {
  //ledcSetup(0, 2000, 8); // פונקציה זו הוחלפה
  //ledcAttachPin(SPEAKER_PIN, 0); // פונקציה זו הוחלפה

  // שימוש בפונקציה החדשה ledcAttach:
  // ledcAttach(pin, frequency, resolution_bits)
  ledcAttach(SPEAKER_PIN, 2000, 8);
}

void loop() {
  // הפעלת צליל 1kHz למשך 500ms
  ledcWrite(SPEAKER_PIN, 128); // עוצמה חצי (הערוץ משויך אוטומטית לפי ה-pin)
  delay(500);
  ledcWrite(SPEAKER_PIN, 0);   // כיבוי צליל
  delay(1000);
}