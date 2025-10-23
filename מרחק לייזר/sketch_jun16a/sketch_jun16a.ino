#include <Wire.h>
#include <Adafruit_VL53L0X.h>
//#define TRIG_PIN 4  // פין TRIG של החיישן מחובר ל-D5
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115500); // חשוב לוודא שגם ה-Serial Monitor מוגדר למהירות זו
  while (!Serial) delay(10); // המתן לחיבור Serial (רלוונטי ללוחות מסוימים כמו Leonardo/ESP32)

  Serial.println("מתחיל בדיקת חיישן Adafruit VL53L0X");

  if (!lox.begin()) {
    Serial.println(F("שגיאה: כשל באתחול חיישן VL53L0X"));
    while(1); // לולאה אינסופית אם החיישן לא עולה
  }
  Serial.println(F("חיישן VL53L0X אותחל בהצלחה!"));
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;
  Serial.print("קורא מדידה... ");
  lox.rangingTest(&measure, false); // מבצע מדידת טווח

  if (measure.RangeStatus != 4) { // Status 4 מציין שהטווח מחוץ לתחום
    float distance_cm = measure.RangeMilliMeter / 10.0; // המרת מילימטרים לסנטימטרים
    Serial.print("מרחק: ");
    Serial.print(distance_cm, 2); // הדפסת המרחק עם 2 ספרות אחרי הנקודה העשרונית
    Serial.println(" ס\"מ");
  } else {
    Serial.println(" מחוץ לטווח מדידה ");
  }

  delay(300); // המתן 100 מילישניות בין מדידה למדידה
}