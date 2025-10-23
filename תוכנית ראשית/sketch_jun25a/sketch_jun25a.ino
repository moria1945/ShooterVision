// --- ספריות נדרשות ---
#include <Wire.h>          // חובה ל-MAX30205 ו-VL53L0X (I2C)
#include <Adafruit_VL53L0X.h> // חובה לחיישן VL53L0X
// #include <DFRobotDFPlayerMini.h> // חובה ל-DFPlayer Mini - הסר את ההערה אם בשימוש

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

// חיישן מרחק לייזר VL53L0X
Adafruit_VL53L0X lox; // יצירת אובייקט לחיישן VL53L0X

// --- הגדרות ל-DFPlayer Mini (אם בשימוש) ---
// **בחר אחת מהאפשרויות הבאות בהתאם לזמינות הפינים בלוח שלך:**
// אפשרות 1: אם פינים 16 ו-17 זמינים (מומלץ אם הם אכן זמינים)
#define DFPLAYER_ESP_TX_PIN 17 // פין TX של ESP32 (ל-RX של DFPlayer)
#define DFPLAYER_ESP_RX_PIN 16 // פין RX של ESP32 (ל-TX של DFPlayer עם נגד 1K)

// אפשרות 2: אם פינים 16 ו-17 אינם זמינים, נסה את 13 ו-12
// #define DFPLAYER_ESP_TX_PIN 13
// #define DFPLAYER_ESP_RX_PIN 12

// HardwareSerial mySerial(2); // אתחול אובייקט Serial2 (UART2) - הסר הערה אם DFPlayer בשימוש
// DFRobotDFPlayerMini myDFPlayer; // הסר הערה אם DFPlayer בשימוש


// --- ידיות (Handles) לתהליכונים (Tasks) ---
// אלו משתנים שמאפשרים לך לשלוט בתהליכונים אם תצטרך (לדוגמה, להשהות או למחוק אותם)
TaskHandle_t TaskMAX30205;
TaskHandle_t TaskPIR;
TaskHandle_t TaskHCSR04;
TaskHandle_t TaskIRObstacle;
TaskHandle_t TaskVL53L0X;
// TaskHandle_t TaskDFPlayer; // הסר הערה אם DFPlayer בשימוש


// --- פונקציות עזר לקריאה מחיישנים (כפי שהיו בקוד המקורי) ---
// פונקציות אלו נקראות על ידי התהליכונים המתאימים
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

  // pulseIn הוא קוד חוסם, אך עבור HC-SR04 הוא הכרחי.
  // הוספת timeout למניעת תקיעות אם אין הד.
  long duration = pulseIn(HCSR04_ECHO_PIN, HIGH, 100000); // Timeout של 100ms
  if (duration == 0) return NAN; // אם אין הד, נחזיר NAN.
  
  float distance_cm = duration * 0.034 / 2;
  return distance_cm;
}


//## פונקציות התהליכונים (Tasks)

//כל פונקציה כזו תרוץ בתהליכון נפרד.

//```cpp
// --- תהליכון חיישן טמפרטורה MAX30205 ---
void max30205Task(void * pvParameters) {
  // Wire.begin() כבר נקרא ב-setup(), אז אין צורך כאן.
  Serial.println("MAX30205 Task: Started.");
  
  for (;;) { // לולאה אינסופית עבור התהליכון
    float temp = readTemperature();
    if (!isnan(temp)) {
      Serial.printf("MAX30205 Task (Core %d): טמפרטורה: %.2f °C\n", xPortGetCoreID(), temp);
    } else {
      Serial.printf("MAX30205 Task (Core %d): שגיאה בקריאת טמפרטורה.\n", xPortGetCoreID());
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); // קריאה כל שנייה
  }
}

// --- תהליכון חיישן תנועה PIR ---
void pirTask(void * pvParameters) {
  pinMode(PIR_PIN, INPUT); // לוודא פין מוגדר כ-INPUT
  Serial.println("PIR Task: Started. Waiting for cooldown (20 seconds)...");
  
  // זמן ה"חימום" והתייצבות של ה-PIR (20 שניות)
  // חשוב לא לחסום את כל המערכת בזמן הזה, אז נשתמש ב-vTaskDelay.
  vTaskDelay(20000 / portTICK_PERIOD_MS); 
  
  Serial.println("PIR Task: Ready.");
  unsigned long lastDetectionMillis = 0; // משתנה מקומי לתהליכון PIR לניהול קירור (cooldown)
  const unsigned long COOLDOWN_MS = 2000; // זמן צינון למניעת זיהויים מהירים מדי

  for (;;) {
    int pirState = digitalRead(PIR_PIN);
    if (pirState == HIGH) {
      if (millis() - lastDetectionMillis >= COOLDOWN_MS) {
        Serial.printf("PIR Task (Core %d): תנועה זוהתה!\n", xPortGetCoreID());
        // myDFPlayer.play(1); // הפעלת קובץ קול 1 (אם DFPlayer בשימוש)
        lastDetectionMillis = millis();
      }
    }
    // קריאת digitalRead היא מהירה מאוד, נשהה קצרות כדי לתת זמן ל-RTOS
    vTaskDelay(50 / portTICK_PERIOD_MS); // קריאה כל 50 מילישניות
  }
}

// --- תהליכון חיישן מרחק אולטרה-סוני (HC-SR04) ---
void hcsr04Task(void * pvParameters) {
  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  Serial.println("HCSR04 Task: Started.");

  for (;;) {
    float distance = readUltrasonicDistanceCm();
    if (!isnan(distance)) {
      Serial.printf("HCSR04 Task (Core %d): מרחק אולטרה-סוני: %.1f ס\"מ\n", xPortGetCoreID(), distance);
    } else {
      Serial.printf("HCSR04 Task (Core %d): שגיאת קריאה/מחוץ לטווח.\n", xPortGetCoreID());
    }
    vTaskDelay(500 / portTICK_PERIOD_MS); // קריאה כל 500 מילישניות
  }
}

// --- תהליכון חיישן אינפרא אדום (IR Obstacle Avoidance) ---
void irObstacleTask(void * pvParameters) {
  pinMode(IR_OBSTACLE_PIN, INPUT);
  Serial.println("IR Obstacle Task: Started.");

  for (;;) {
    int obstacleDetected = digitalRead(IR_OBSTACLE_PIN);
    // HIGH = מכשול עבור החיישן שלך (ודא זאת לפי המפרט הטכני של החיישן הספציפי)
    if (obstacleDetected == HIGH) {
      Serial.printf("IR Obstacle Task (Core %d): מכשול IR זוהה!\n", xPortGetCoreID());
    } else {
      Serial.printf("IR Obstacle Task (Core %d): אין מכשול IR.\n", xPortGetCoreID());
    }
    vTaskDelay(200 / portTICK_PERIOD_MS); // קריאה כל 200 מילישניות
  }
}

// --- תהליכון חיישן מרחק לייזר VL53L0X ---
void vl53l0xTask(void * pvParameters) {
  Serial.println("VL53L0X Task: Initializing sensor...");
  // אתחול החיישן בתוך התהליכון. Wire.begin() חייב להיות ב-setup().
  if (!lox.begin()) {
    Serial.println(F("VL53L0X Task: שגיאה: כשל באתחול חיישן VL53L0X. ודא חיבורים תקינים."));
    Serial.println(F("VL53L0X Task: ודא גם שאתה משתמש בספק כוח חיצוני מספק לכל המערכת."));
    while(1); // לולאה אינסופית אם החיישן לא עולה - תהליכון זה יתקע כאן.
  }
  Serial.println(F("VL53L0X Task: חיישן VL53L0X אותחל בהצלחה!"));

  for (;;) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false); // קריאת מרחק

    if (measure.RangeStatus != 4) {  // 4 פירושו "מעבר לטווח"
      float distance_cm = measure.RangeMilliMeter / 10.0;
      Serial.printf("VL53L0X Task (Core %d): מרחק לייזר: %.2f ס\"מ\n", xPortGetCoreID(), distance_cm);
    } else {
      Serial.printf("VL53L0X Task (Core %d): מחוץ לטווח מדידה.\n", xPortGetCoreID());
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // קריאה כל 100 מילישניות
  }
}

// --- תהליכון DFPlayer Mini (אם בשימוש) ---
/*
// הסר את הערה והגדר את mySerial ו-myDFPlayer למעלה אם בשימוש
HardwareSerial mySerial(2); // אתחול אובייקט Serial2 (UART2)
DFRobotDFPlayerMini myDFPlayer;

void dfPlayerTask(void * pvParameters) {
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_ESP_RX_PIN, DFPLAYER_ESP_TX_PIN); // RX, TX
  Serial.println(F("DFPlayer Task: אתחול DFPlayer Mini..."));
  if (!myDFPlayer.begin(mySerial)) {
    Serial.println(F("DFPlayer Task: שגיאה: כשל באתחול DFPlayer Mini. ודא חיבורים."));
    while(1);
  }
  Serial.println(F("DFPlayer Task: DFPlayer Mini אותחל בהצלחה!"));
  myDFPlayer.volume(20); // הגדר ווליום (0-30), 20 זה בינוני-גבוה

  for (;;) {
    // כאן תוכל להוסיף לוגיקה להפעלת קבצי אודיו.
    // לדוגמה, אתה יכול להשתמש ב"דגל" (משתנה בוליאני גלובלי) שתהליכון PIR מגדיר ל-true,
    // ותהליכון ה-DFPlayer יבדוק את הדגל וישמיע צליל.
    // myDFPlayer.loop(); // אם יש צורך בלולאת עיבוד פנימית של הספריה
    vTaskDelay(100 / portTICK_PERIOD_MS); // השהייה קבועה כדי לא לתפוס את ה-CPU
  }
}
*/


//## פונקציית SETUP ראשית

void setup() {
  // אתחול תקשורת טורית ראשית (ל-Serial Monitor)
  Serial.begin(115200);
  while (!Serial) delay(10); // המתן לחיבור Serial
  Serial.println("\n--- מערכת חיישנים מאוחדת מתחילה ---");

  // אתחול I2C (עבור MAX30205 ו-VL53L0X)
  // זה חייב לקרות פעם אחת לפני שכל תהליכון I2C ינסה לאתחל את החיישן שלו.
  Wire.begin(); 
  Serial.println("Main Setup: I2C initialized.");
  
  // יצירת התהליכונים עבור כל חיישן
  // מומלץ להריץ את התהליכונים על CORE1 (ליבת היישום - APP_CPU_NUM)
  // כדי לא להפריע למשימות ה-WiFi והמערכת שרצות על CORE0 (PRO_CPU_NUM).
  // גדלי מחסנית (Stack size) הם הערכות בבתים (Bytes), ייתכן שיש צורך לכוונן אותם.
  // אם אתה מקבל שגיאות כמו "Stack overflow", הגדל את המספר.

  xTaskCreatePinnedToCore(
    max30205Task,   /* הפונקציה שתרוץ כתהליכון */
    "MAX30205_Task",/* שם קריא לתהליכון (לניפוי באגים) */
    5000,           /* גודל מחסנית בבתים (Stack size in bytes) */
    NULL,           /* פרמטרים שיועברו לפונקציה (NULL אם אין) */
    1,              /* עדיפות התהליכון (1 הוא נמוך, גבוה יותר חשוב יותר) */
    &TaskMAX30205,  /* מצביע לתהליכון שנוצר (ידית) */
    APP_CPU_NUM     /* ליבה לריצה (Core 1 - ליבת היישום) */
  );

  xTaskCreatePinnedToCore(
    pirTask,        /* הפונקציה שתרוץ כתהליכון */
    "PIR_Task",     /* שם לתהליכון */
    3000,           /* גודל מחסנית בבתים */
    NULL,           /* פרמטרים */
    1,              /* עדיפות */
    &TaskPIR,       /* ידית */
    APP_CPU_NUM     /* ליבה */
  );

  xTaskCreatePinnedToCore(
    hcsr04Task,     /* הפונקציה שתרוץ כתהליכון */
    "HCSR04_Task",  /* שם לתהליכון */
    4000,           /* גודל מחסנית בבתים */
    NULL,           /* פרמטרים */
    1,              /* עדיפות */
    &TaskHCSR04,    /* ידית */
    APP_CPU_NUM     /* ליבה */
  );

  xTaskCreatePinnedToCore(
    irObstacleTask, /* הפונקציה שתרוץ כתהליכון */
    "IR_Task",      /* שם לתהליכון */
    3000,           /* גודל מחסנית בבתים */
    NULL,           /* פרמטרים */
    1,              /* עדיפות */
    &TaskIRObstacle,/* ידית */
    APP_CPU_NUM     /* ליבה */
  );

  xTaskCreatePinnedToCore(
    vl53l0xTask,    /* הפונקציה שתרוץ כתהליכון */
    "VL53L0X_Task", /* שם לתהליכון */
    10000,          /* גודל מחסנית בבתים (VL53L0X עשויה לדרוש יותר בגלל הספריה) */
    NULL,           /* פרמטרים */
    1,              /* עדיפות */
    &TaskVL53L0X,   /* ידית */
    APP_CPU_NUM     /* ליבה */
  );
  
  // אם אתה משתמש ב-DFPlayer Mini, הסר את הערה מהקוד הזה ומהבלוקים המתאימים למעלה.
  /*
  xTaskCreatePinnedToCore(
    dfPlayerTask,     // הפונקציה שתרוץ
    "DFPlayer_Task",  // שם לתהליכון
    4000,             // גודל מחסנית בבתים
    NULL,             // פרמטרים
    1,                // עדיפות
    &TaskDFPlayer,    // ידית
    APP_CPU_NUM       // ליבה
  );
  */

  Serial.println("--- כל התהליכונים נוצרו בהצלחה. ---");
  Serial.println("הפונקציה loop() תמשיך לרוץ במקביל או תישאר כמעט ריקה.");
}


//## פונקציית LOOP ראשית (מינימלית)
void loop() {
  // בפרויקטים מבוססי FreeRTOS, פונקציית loop() הופכת לעיתים קרובות לריקה
  // או שהיא מכילה רק משימות רקע קלות שאינן דורשות תהליכון נפרד.
  // היא עצמה מהווה תהליכון בפני עצמה (ברירת מחדל על Core 1 ב-ESP32).
  // חשוב לשים כאן vTaskDelay קצר כדי לא לבזבז זמן CPU אם אין לוגיקה משמעותית,
  // ולאפשר למתזמן של FreeRTOS לתזמן תהליכונים אחרים.
  vTaskDelay(100 / portTICK_PERIOD_MS); // השהייה קצרה של 100 מילישניות
}