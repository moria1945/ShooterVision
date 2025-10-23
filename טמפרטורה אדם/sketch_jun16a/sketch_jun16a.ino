// --- ספריות נדרשות ---
#include <Wire.h>          // חובה ל-MAX30205 ו-VL53L0X (I2C)
#include <Adafruit_VL53L0X.h> // חובה לחיישן VL53L0X
#include <DFRobotDFPlayerMini.h> // חובה ל-DFPlayer Mini

// --- הגדרות פינים ---
// חיישן טמפרטורה MAX30205
#define MAX30205_ADDRESS 0x48 // כתובת I2C של MAX30205
// I2C SCL (Clock) ו-SDA (Data) מוגדרים אוטומטית ב-Wire.begin() ב-ESP32 כ-GPIO22 ו-GPIO21 בהתאמה.

// חיישן תנועה PIR
#define PIR_PIN 32 // פין OUT של חיישן PIR

// חיישן מרחק אולטרה-סוני (HC-SR04)
#define HCSR04_TRIG_PIN 5  // פין TRIG של חיישן HC-SR04
#define HCSR04_ECHO_PIN 18 // פין ECHO של חיישן HC-SR04

// חיישן אינפרא אדום (IR Obstacle Avoidance)
#define IR_OBSTACLE_PIN 27 // פין IR: עבר לפין 27 למניעת התנגשויות

// חיישני מרחק לייזר VL53L0X
// VL53L0X_A ו-VL53L0X_B יחוברו שניהם ל-I2C (GPIO21, GPIO22).
// כל אחד מהם יקבל פין XSHUT נפרד.
#define VL53L0X_A_XSHUT_PIN 14 // פין XSHUT עבור VL53L0X הראשון (VL53L0X_A)
#define VL53L0X_B_XSHUT_PIN 15 // פין XSHUT עבור VL53L0X השני (VL53L0X_B)

// יצירת אובייקטים לחיישני VL53L0X.
// כל אובייקט צריך להיות מוגדר בנפרד.
Adafruit_VL53L0X lox_a;
Adafruit_VL53L0X lox_b;

// --- הגדרות ל-DFPlayer Mini ---
// **בחר אחת מהאפשרויות הבאות בהתאם לזמינות הפינים בלוח שלך:**

// אפשרות 1: אם פינים 16 ו-17 זמינים (מומלץ אם הם אכן זמינים עבור UART2)
#define DFPLAYER_ESP_TX_PIN 17 // פין TX של ESP32 (מחובר ל-RX של DFPlayer)
#define DFPLAYER_ESP_RX_PIN 16 // פין RX של ESP32 (מחובר ל-TX של DFPlayer **עם נגד 1K אום בטור**)

// אפשרות 2: אם פינים 16 ו-17 אינם זמינים, נסה את 13 ו-12 (פחות נפוץ שהם תפוסים)
// **הערה: אם אתה משתמש באפשרות זו, שנה את שורות ההגדרה שמעל לקומנט (//) והסר את הקומנט מהשורות הבאות**
//#define DFPLAYER_ESP_TX_PIN 13 // פין TX של ESP32 (מחובר ל-RX של DFPlayer)
//#define DFPLAYER_ESP_RX_PIN 12 // פין RX של ESP32 (מחובר ל-TX של DFPlayer **עם נגד 1K אום בטור**)

HardwareSerial mySerial(2); // אתחול אובייקט Serial2 (UART2)
DFRobotDFPlayerMini myDFPlayer;


// --- משתנים גלובליים לניהול זמנים ---
const unsigned long PIR_DETECTION_COOLDOWN_MS = 3000;
unsigned long lastPirDetectionTime = 0;

unsigned long lastUltrasonicReadTime = 0;
const unsigned long ULTRASONIC_READ_INTERVAL_MS = 500;

unsigned long lastIrReadTime = 0;
const unsigned long IR_READ_INTERVAL_MS = 200;

unsigned long lastTempReadTime = 0;
const unsigned long TEMP_READ_INTERVAL_MS = 1000;

unsigned long lastVl53l0xReadTime = 0;
const unsigned long VL53L0X_READ_INTERVAL_MS = 100;


// --- פונקציות קריאה לחיישנים ---

float readTemperature() {
  Wire.beginTransmission(MAX30205_ADDRESS);
  Wire.write(0x00); // Register to read temperature
  Wire.endTransmission(false); // don't release bus
  Wire.requestFrom(MAX30205_ADDRESS, 2);

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int16_t tempRaw = (msb << 8) | lsb;
    float temperature = tempRaw * 0.00390625; // According to datasheet
    return temperature;
  }
  return NAN; // Not A Number - to indicate error
}

float readUltrasonicDistanceCm() {
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, LOW);

  long duration = pulseIn(HCSR04_ECHO_PIN, HIGH);
  // Calculate distance in cm (speed of sound 0.034 cm/us)
  float distance_cm = duration * 0.034 / 2;
  return distance_cm;
}


// --- פונקציית SETUP ראשית ---
void setup() {
  // אתחול תקשורת טורית ראשית (ל-Serial Monitor)
  Serial.begin(115200);
  while (!Serial) delay(10); // המתן לחיבור Serial (לא תמיד הכרחי ב-ESP32, אבל לא מזיק)
  Serial.println("--- מערכת חיישנים מאוחדת מתחילה ---");

  // אתחול I2C (עבור MAX30205 ו-VL53L0X)
  Wire.begin(); // ESP32 משתמש כברירת מחדל ב-GPIO21 (SDA) ו-GPIO22 (SCL)

  // הגדרת פינים לחיישנים
  pinMode(PIR_PIN, INPUT);
  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  pinMode(IR_OBSTACLE_PIN, INPUT); // עבור חיישן ה-IR

  // הגדרת פיני XSHUT עבור VL53L0X
  pinMode(VL53L0X_A_XSHUT_PIN, OUTPUT);
  pinMode(VL53L0X_B_XSHUT_PIN, OUTPUT);

  // להוריד את כל פיני ה-XSHUT ל-LOW כדי לוודא שכל החיישנים כבויים
  // (כתובת ברירת המחדל שלהם 0x29)
  digitalWrite(VL53L0X_A_XSHUT_PIN, LOW);
  digitalWrite(VL53L0X_B_XSHUT_PIN, LOW);
  delay(10); // לתת לחיישנים זמן להתייצב במצב כבוי

  Serial.println("מתחיל בדיקת חיישני Adafruit VL53L0X...");

  // אתחול VL53L0X_A (חיישן ראשון)
  // 1. העלה את XSHUT של חיישן A ל-HIGH (כדי להפעיל אותו)
  digitalWrite(VL53L0X_A_XSHUT_PIN, HIGH);
  delay(10); // תן לחיישן זמן להתעורר

  // 2. אתחל את חיישן A ושנה את כתובת ה-I2C שלו לכתובת חדשה
  Serial.println(F("מאתחל VL53L0X_A..."));
  if (!lox_a.begin()) {
    Serial.println(F("שגיאה: כשל באתחול VL53L0X_A. ודא חיבורי I2C ו-XSHUT תקינים."));
    while(1);
  }
  // שנה את הכתובת ל-0x30 (כל כתובת חופשית שאינה 0x29)
  lox_a.setAddress(0x30);
  Serial.println(F("VL53L0X_A אותחל בהצלחה וכתובתו שונתה ל-0x30!"));
  delay(100); // תן לחיישן זמן לעבד את שינוי הכתובת

  // אתחול VL53L0X_B (חיישן שני)
  // 1. העלה את XSHUT של חיישן B ל-HIGH (כדי להפעיל אותו)
  digitalWrite(VL53L0X_B_XSHUT_PIN, HIGH);
  delay(10); // תן לחיישן זמן להתעורר

  // 2. אתחל את חיישן B ושנה את כתובת ה-I2C שלו לכתובת חדשה
  Serial.println(F("מאתחל VL53L0X_B..."));
  if (!lox_b.begin()) {
    Serial.println(F("שגיאה: כשל באתחול VL53L0X_B. ודא חיבורי I2C ו-XSHUT תקינים."));
    while(1);
  }
  // שנה את הכתובת ל-0x31 (כל כתובת חופשית אחרת)
  lox_b.setAddress(0x31);
  Serial.println(F("VL53L0X_B אותחל בהצלחה וכתובתו שונתה ל-0x31!"));
  delay(100); // תן לחיישן זמן לעבד את שינוי הכתובת

  Serial.println(F("כל חיישני VL53L0X אותחלו בהצלחה!"));


  // אתחול DFPlayer Mini
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_ESP_RX_PIN, DFPLAYER_ESP_TX_PIN); // RX, TX
  Serial.println(F("אתחול DFPlayer Mini..."));
  if (!myDFPlayer.begin(mySerial)) {
    Serial.println(F("שגיאה: כשל באתחול DFPlayer Mini. ודא חיבורים (TX->RX, RX->TX עם נגד 1K, VCC, GND)."));
    Serial.println(F("ודא שפיני ה-UART זמינים (בהתאם לבחירתך למעלה). בדוק כרטיס SD וקבצים."));
    while(1);
  }
  Serial.println(F("DFPlayer Mini אותחל בהצלחה!"));
  myDFPlayer.volume(20); // הגדר ווליום (0-30), 20 זה בינוני-גבוה

  // הודעות אתחול PIR
  Serial.println("ממתין לאתחול חיישן PIR (20 שניות)...");
  delay(20000); // זמן "חימום" והתייצבות ל-PIR
  Serial.println("חיישן PIR מוכן.");

  Serial.println("--- אתחול חיישנים הסתיים ---");
}


// --- פונקציית LOOP ראשית ---
void loop() {
  unsigned long currentTime = millis(); // קבל את הזמן הנוכחי

  // --- קריאת חיישן טמפרטורה MAX30205 ---
  if (currentTime - lastTempReadTime >= TEMP_READ_INTERVAL_MS) {
    float temp = readTemperature();
    if (!isnan(temp)) {
      Serial.print("טמפרטורה (MAX30205): ");
      Serial.print(temp);
      Serial.println(" °C");
    } else {
      Serial.println("שגיאה בקריאת טמפרטורה (MAX30205)");
    }
    lastTempReadTime = currentTime;
  }

  // --- קריאת חיישן תנועה PIR ---
  int pirState = digitalRead(PIR_PIN);
  if (pirState == HIGH) {
    if (currentTime - lastPirDetectionTime >= PIR_DETECTION_COOLDOWN_MS) {
      Serial.println("תנועה זוהתה!");
      //myDFPlayer.play(1); // דוגמה: הפעלת קובץ קול מספר 1 מכרטיס SD
      lastPirDetectionTime = currentTime;
    }
  } else {
    // ניתן להדפיס "אין תנועה" רק אם רוצים, אך זה יכול להציף את ה-Serial Monitor
    // Serial.println("אין תנועה.");
  }

  // --- קריאת חיישן מרחק אולטרה-סוני HC-SR04 ---
  if (currentTime - lastUltrasonicReadTime >= ULTRASONIC_READ_INTERVAL_MS) {
    float distance = readUltrasonicDistanceCm();
    Serial.print("מרחק (אולטרה-סוני): ");
    Serial.print(distance);
    Serial.println(" ס\"מ");
    lastUltrasonicReadTime = currentTime;
  }

  // --- קריאת חיישן IR לזיהוי מכשולים ---
  if (currentTime - lastIrReadTime >= IR_READ_INTERVAL_MS) {
    int obstacleDetected = digitalRead(IR_OBSTACLE_PIN);
    // HIGH = מכשול עבור החיישן שלך
    if (obstacleDetected == HIGH) {
      Serial.println("מכשול IR זוהה!");
    } else {
      Serial.println("אין מכשול IR");
    }
    lastIrReadTime = currentTime;
  }

  // --- קריאת חיישני מרחק לייזר VL53L0X ---
  if (currentTime - lastVl53l0xReadTime >= VL53L0X_READ_INTERVAL_MS) {
    VL53L0X_RangingMeasurementData_t measure;

    // קריאה מחיישן VL53L0X_A
    lox_a.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      float distance_cm = measure.RangeMilliMeter / 10.0;
      Serial.print("מרחק לייזר (VL53L0X_A): ");
      Serial.print(distance_cm, 2);
      Serial.println(" ס\"מ");
    } else {
      Serial.println("מרחק לייזר (VL53L0X_A): מחוץ לטווח מדידה");
    }

    // קריאה מחיישן VL53L0X_B
    lox_b.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      float distance_cm = measure.RangeMilliMeter / 10.0;
      Serial.print("מרחק לייזר (VL53L0X_B): ");
      Serial.print(distance_cm, 2);
      Serial.println(" ס\"מ");
    } else {
      Serial.println("מרחק לייזר (VL53L0X_B): מחוץ לטווח מדידה");
    }
    lastVl53l0xReadTime = currentTime;
  }

  // השהייה קצרה כדי לא להעמיס על ה-CPU ועל ה-Serial Monitor
  delay(10);
}