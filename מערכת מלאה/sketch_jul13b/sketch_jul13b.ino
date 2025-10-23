// --- ספריות נדרשות ---
#include <Wire.h>          // חובה ל-MAX30205 ו-VL53L0X (I2C)
#include <Adafruit_VL53L0X.h> // חובה לחיישן VL53L0X
#include <DFRobotDFPlayerMini.h> // חובה ל-DFPlayer Mini

// --- הגדרות פינים ---
// חיישן טמפרטורה MAX30205
#define MAX30205_ADDRESS 0x48

// חיישן תנועה PIR
#define PIR_PIN 32

// חיישן מרחק אולטרה-סוני (HC-SR04)
#define HCSR04_TRIG_PIN 5
#define HCSR04_ECHO_PIN 18

// חיישן אינפרא אדום (IR Obstacle Avoidance)
#define IR_OBSTACLE_PIN 27

// חיישן מרחק לייזר VL53L0X
Adafruit_VL53L0X lox;
bool vl53l0x_initialized = false; // דגל כדי לדעת אם החיישן אותחל בהצלחה

// --- הגדרות ל-DFPlayer Mini ---
#define DFPLAYER_ESP_TX_PIN 17 // פין TX של ESP32 (ל-RX של DFPlayer)
#define DFPLAYER_ESP_RX_PIN 16 // פין RX של ESP32 (ל-TX של DFPlayer עם נגד 1K)

HardwareSerial mySerial(2); // אתחול אובייקט Serial2 (UART2)
DFRobotDFPlayerMini myDFPlayer;

// --- הגדרות צלילים ב-DFPlayer Mini ---
// ודא שקבצים אלה קיימים בכרטיס ה-SD שלך (למשל, בתיקיית mp3/)
#define SOUND_WARNING_APPROACHING 1 // קובץ שיר מספר 1 (לדוגמה, "אזהרה!")
#define SOUND_GUNSHOT_FIRING      2 // קובץ שיר מספר 2 (לדוגמה, "קול ירייה")

// --- הגדרות לזמזם הכחול (Buzzer) ---
#define BUZZER_PIN 25 // פין הזמזם הכחול
#define BUZZER_PWM_FREQ 2000 // תדר PWM (הרץ) לצליל
#define BUZZER_PWM_RESOLUTION 8 // רזולוציית PWM (ביטים)
#define BUZZER_TONE_ON_VALUE 128 // ערך עוצמה לצליל (חצי מרזולוציה 8 ביט)

// --- ידיות (Handles) לתהליכונים (Tasks) ---
TaskHandle_t TaskMAX30205;
TaskHandle_t TaskPIR;
TaskHandle_t TaskHCSR04;
TaskHandle_t TaskIRObstacle;
TaskHandle_t TaskDFPlayer;
TaskHandle_t TaskDecisionMaking; 

// --- תור הודעות לתקשורת בין תהליכונים ---
QueueHandle_t dfPlayerQueue;

// --- מבנה נתונים משותף לנתוני חיישנים ---
typedef struct {
  bool pirDetected;
  bool irObstacleDetected;
  float vl53l0xDistanceCm;
  float hcsr04DistanceCm;
  float temperatureC;
} SensorData;

SensorData currentSensorData; 
SemaphoreHandle_t xSensorDataMutex; 

// --- הגדרות ספים למרחק ---
const float THRESHOLD_WARNING_CM = 10.0; 
const float THRESHOLD_FIRING_CM = 5.0;   
const float MAX_DISTANCE_VALID_CM = 500.0; 

// --- פונקציות עזר לקריאה מחיישנים ---
float readTemperature() { // טמפרטורה
  Wire.beginTransmission(MAX30205_ADDRESS);
  Wire.write(0x00);
  Wire.endTransmission(false);
  Wire.requestFrom(MAX30205_ADDRESS, 2);

  if (Wire.available() == 2) {
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();
    int16_t tempRaw = (msb << 8) | lsb;
    float temperature = tempRaw * 0.00390625;
    return temperature;
  }
  return NAN;
}

float readUltrasonicDistanceCm() {
  digitalWrite(HCSR04_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(HCSR04_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(HCSR04_TRIG_PIN, LOW);

  long duration = pulseIn(HCSR04_ECHO_PIN, HIGH, 20000); 
  if (duration == 0) return NAN;
  
  float distance_cm = duration * 0.034 / 2;
  return distance_cm;
}

float readVL53L0XDistanceCm() { //מרחק ליזר
  if (!vl53l0x_initialized) {
    return MAX_DISTANCE_VALID_CM + 1; 
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  
  if (measure.RangeStatus != 4) { 
    return measure.RangeMilliMeter / 10.0;
  } else {
    return MAX_DISTANCE_VALID_CM + 1; 
  }
}

// --- פונקציות התהליכונים (Tasks) ---

void max30205Task(void * pvParameters) {
  Serial.println("MAX30205 Task: Started.");
  for (;;) {
    float temp = readTemperature();
    if (!isnan(temp)) {
      if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) { 
        currentSensorData.temperatureC = temp;
        xSemaphoreGive(xSensorDataMutex); 
      }
      Serial.printf("MAX30205 Task (Core %d): טמפרטורה: %.2f °C🌡️\n", xPortGetCoreID(), temp); 
    } else {
      if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
        currentSensorData.temperatureC = NAN; 
        xSemaphoreGive(xSensorDataMutex);
      }
      Serial.printf("MAX30205 Task (Core %d): שגיאה בקריאת טמפרטורה. ❌\n", xPortGetCoreID()); 
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS); 
  }
}

void pirTask(void * pvParameters) {
  pinMode(PIR_PIN, INPUT);
  Serial.println("PIR Task: Started. Waiting for cooldown (20 seconds)...");
  vTaskDelay(20000 / portTICK_PERIOD_MS); 
  Serial.println("PIR Task: Ready.");
  
  int lastPirState = LOW; 
  unsigned long lastDetectionPrintMillis = 0;
  const unsigned long COOLDOWN_PRINT_MS = 1000; 

  for (;;) {
    int pirState = digitalRead(PIR_PIN);
    bool detected = (pirState == HIGH);
    
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      currentSensorData.pirDetected = detected;
      xSemaphoreGive(xSensorDataMutex);
    }

    if (pirState != lastPirState) { 
      if (pirState == HIGH) {
        Serial.printf("PIR Task (Core %d): תנועה זוהתה! 🚶‍♂️\n", xPortGetCoreID()); 
        lastDetectionPrintMillis = millis(); 
      } else {
        Serial.printf("PIR Task (Core %d): תנועה הסתיימה. 🛑\n", xPortGetCoreID()); 
      }
      lastPirState = pirState; 
    } else if (pirState == HIGH && (millis() - lastDetectionPrintMillis >= COOLDOWN_PRINT_MS)) {
      Serial.printf("PIR Task (Core %d): עדיין יש תנועה... 🏃‍♀️\n", xPortGetCoreID()); 
      lastDetectionPrintMillis = millis();
    }

    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void hcsr04Task(void * pvParameters) {
  pinMode(HCSR04_TRIG_PIN, OUTPUT);
  pinMode(HCSR04_ECHO_PIN, INPUT);
  Serial.println("HCSR04 Task: Started.");

  for (;;) {
    float distance = readUltrasonicDistanceCm();
    if (!isnan(distance)) {
      if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
        currentSensorData.hcsr04DistanceCm = distance;
        xSemaphoreGive(xSensorDataMutex);
      }
      Serial.printf("HCSR04 Task (Core %d): מרחק אולטרה-סוני: %.1f ס\"מ 📏\n", xPortGetCoreID(), distance); 
    } else {
      if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
        currentSensorData.hcsr04DistanceCm = MAX_DISTANCE_VALID_CM + 1; 
        xSemaphoreGive(xSensorDataMutex);
      }
      Serial.printf("HCSR04 Task (Core %d): שגיאת קריאה/מחוץ לטווח. ❌\n", xPortGetCoreID()); 
    }
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

void irObstacleTask(void * pvParameters) {
  pinMode(IR_OBSTACLE_PIN, INPUT);
  Serial.println("IR Obstacle Task: Started.");

  for (;;) {
    int obstaclePinState = digitalRead(IR_OBSTACLE_PIN);
    bool detected = (obstaclePinState == LOW); 
    
    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      currentSensorData.irObstacleDetected = detected;
      xSemaphoreGive(xSensorDataMutex);
    }

    if (detected) {
      Serial.printf("IR Obstacle Task (Core %d): מכשול IR זוהה! 🚧\n", xPortGetCoreID()); 
    } else {
      Serial.printf("IR Obstacle Task (Core %d): אין מכשול IR. ✅\n", xPortGetCoreID()); 
    }
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void dfPlayerTask(void * pvParameters) {
  mySerial.begin(9600, SERIAL_8N1, DFPLAYER_ESP_RX_PIN, DFPLAYER_ESP_TX_PIN);
  Serial.println(F("DFPlayer Task: אתחול DFPlayer Mini..."));
  if (!myDFPlayer.begin(mySerial, true, false)) { 
    Serial.println(F("DFPlayer Task: שגיאה: כשל באתחול DFPlayer Mini. ודא חיבורים. ⚠️")); 
  } else {
    Serial.println(F("DFPlayer Task: DFPlayer Mini אותחל בהצלחה! ✅")); 
    myDFPlayer.volume(20); 
    myDFPlayer.setTimeOut(500); 
  }

  uint8_t receivedSoundId;

  for (;;) {
    if (xQueueReceive(dfPlayerQueue, &receivedSoundId, portMAX_DELAY) == pdTRUE) {
      Serial.printf("DFPlayer Task: קיבל פקודה לנגן שיר %d 🎶\n", receivedSoundId); 
      if (myDFPlayer.readState() == 0x00) { 
         myDFPlayer.play(receivedSoundId);
         Serial.printf("DFPlayer Task: מנגן צליל %d 🔊\n", receivedSoundId); 
      } else {
        Serial.printf("DFPlayer Task: נגן לא מוכן (קוד מצב: 0x%X), לא מנגן. ❌\n", myDFPlayer.readState()); 
      }
     
    }
    vTaskDelay(10 / portTICK_PERIOD_MS); 
  }
}

void decisionMakingTask(void * pvParameters) {
  Serial.println("Decision Making Task: Started.");
  
  bool currentWarningStateActive = false; 
  bool currentFiringStateActive = false;  

  for (;;) {
    float vl53l0x_dist = readVL53L0XDistanceCm(); 

    if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
      currentSensorData.vl53l0xDistanceCm = vl53l0x_dist; 

      bool motionOrObstacleDetected = currentSensorData.pirDetected || currentSensorData.irObstacleDetected;
      float closestDistanceCm = MAX_DISTANCE_VALID_CM + 1; 
      
      if (vl53l0x_initialized && !isnan(currentSensorData.vl53l0xDistanceCm) && currentSensorData.vl53l0xDistanceCm <= MAX_DISTANCE_VALID_CM) {
        closestDistanceCm = min(closestDistanceCm, currentSensorData.vl53l0xDistanceCm);
      }
      if (!isnan(currentSensorData.hcsr04DistanceCm) && currentSensorData.hcsr04DistanceCm <= MAX_DISTANCE_VALID_CM) {
        closestDistanceCm = min(closestDistanceCm, currentSensorData.hcsr04DistanceCm);
      }
      
      xSemaphoreGive(xSensorDataMutex); 

      String unifiedIcon = ""; 
      uint8_t soundToPlay = 0; 

      // לוגיקת קבלת ההחלטות
      // מצב ירייה - עדיפות עליונה (מתחת ל-5 ס"מ)
      if (motionOrObstacleDetected && closestDistanceCm <= THRESHOLD_FIRING_CM) {
        unifiedIcon = " ⚠️"; // אייקון אחיד למצב ירייה: אזהרה/סכנה
        if (!currentFiringStateActive) { 
          Serial.printf("Decision Task: *** אובייקט קרוב מאוד + זיהוי! מצב: ירייה ***%s\n", unifiedIcon.c_str()); 
          soundToPlay = SOUND_GUNSHOT_FIRING;
          
          // --- הפעלת הזמזם הכחול למצב ירייה ---
          // ledcWrite(BUZZER_PWM_CHANNEL, BUZZER_TONE_ON_VALUE); // השורה שהייתה לפני השינוי
          // אין צורך לציין ערוץ כאן ב-ledcAttach, הוא מטפל בזה אוטומטית.
          // אם נצטרך שליטה מדויקת יותר על ערוץ ה-PWM, נצטרך לחזור ל-ledcSetup + ledcAttachPin
          // אבל כרגע, ledcAttach משייך פין לערוץ באופן אוטומטי ואז ledcWrite עובד על הפין.
          ledcWrite(BUZZER_PIN, BUZZER_TONE_ON_VALUE); // הפעלת צליל מהזמזם (באמצעות הפין)
          
          currentFiringStateActive = true;
          currentWarningStateActive = false; 
        }
      } 
      // מצב אזהרה / גבול (מתחת ל-10 ס"מ, אך מעל 5 ס"מ)
      else if (motionOrObstacleDetected && closestDistanceCm <= THRESHOLD_WARNING_CM) {
        unifiedIcon = " 🛑"; // אייקון אחיד למצב אזהרה: עצור/גבול
        if (!currentWarningStateActive && !currentFiringStateActive) { 
          Serial.printf("Decision Task: *** אובייקט מתקרב + זיהוי! מצב: אזהרה ***%s\n", unifiedIcon.c_str()); 
          soundToPlay = SOUND_WARNING_APPROACHING;
          currentWarningStateActive = true;
          currentFiringStateActive = false; 
        }
      } 
      // מצב רגיל / אובייקט התרחק
      else {
        // אם יצאנו ממצב קריטי (ירייה או אזהרה)
        if (currentWarningStateActive || currentFiringStateActive) { 
          Serial.println("Decision Task: אובייקט התרחק או אין זיהוי. מצב: רגיל. ✅"); 
          // --- כיבוי הזמזם במצב רגיל ---
          ledcWrite(BUZZER_PIN, 0); // כיבוי צליל מהזמזם (באמצעות הפין)
        }
        currentWarningStateActive = false;
        currentFiringStateActive = false;
      }

      // שלח פקודת צליל ל-DFPlayer רק אם יש צליל לנגן
      if (soundToPlay != 0) {
        xQueueSend(dfPlayerQueue, &soundToPlay, 0); 
      }

      // הדפסת הודעת VL53L0X עם האייקון האחיד
      if (vl53l0x_initialized) {
        if (vl53l0x_dist <= MAX_DISTANCE_VALID_CM) {
          Serial.printf("Decision Task: מרחק לייזר (VL53L0X): %.2f ס\"מ%s\n", vl53l0x_dist, unifiedIcon.c_str()); 
        } else {
          Serial.printf("Decision Task: VL53L0X: מחוץ לטווח.\n"); 
        }
      } else {
         Serial.printf("Decision Task: VL53L0X אינו זמין (לא אותחל).\n");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS); 
  }
}

// --- פונקציית SETUP ראשית ---
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  Serial.println("\n--- מערכת חיישנים מאוחדת מתחילה ---");

  Wire.begin(); 
  Serial.println("Main Setup: I2C initialized.");

  Serial.println(F("Main Setup: מנסה לאתחל חיישן VL53L0X..."));
  if (!lox.begin()) {
    Serial.println(F("Main Setup: שגיאה: כשל באתחול VL53L0X. נתונים מחיישן זה לא ישמשו. ❌")); 
    vl53l0x_initialized = false;
  } else {
    Serial.println(F("Main Setup: VL53L0X אותחל בהצלחה! ✅")); 
    vl53l0x_initialized = true;
  }
  
  // --- אתחול הזמזם הכחול (מתוקן ל-ledcAttach) ---
  ledcAttach(BUZZER_PIN, BUZZER_PWM_FREQ, BUZZER_PWM_RESOLUTION); // שינוי חשוב כאן!
  ledcWrite(BUZZER_PIN, 0); // וודא שהזמזם כבוי בהתחלה
  Serial.printf("Main Setup: Buzzer on pin %d initialized. 🎵\n", BUZZER_PIN);


  currentSensorData.pirDetected = false;
  currentSensorData.irObstacleDetected = false;
  currentSensorData.vl53l0xDistanceCm = MAX_DISTANCE_VALID_CM + 1; 
  currentSensorData.hcsr04DistanceCm = MAX_DISTANCE_VALID_CM + 1; 
  currentSensorData.temperatureC = NAN;

  xSensorDataMutex = xSemaphoreCreateMutex();
  if (xSensorDataMutex == NULL) {
    Serial.println("שגיאה: כשל ביצירת Mutex. תקוע. ❌"); 
    while(1);
  }

  dfPlayerQueue = xQueueCreate(5, sizeof(uint8_t)); 
  if (dfPlayerQueue == NULL) {
    Serial.println("שגיאה: כשל ביצירת תור DFPlayer. תקוע. ❌"); 
    while(1);
  }

  xTaskCreatePinnedToCore(max30205Task, "MAX30205_Task", 5000, NULL, 1, &TaskMAX30205, APP_CPU_NUM);//טמפרטורה
  xTaskCreatePinnedToCore(pirTask, "PIR_Task", 3000, NULL, 1, &TaskPIR, APP_CPU_NUM);//תנועה
  xTaskCreatePinnedToCore(hcsr04Task, "HCSR04_Task", 4000, NULL, 1, &TaskHCSR04, APP_CPU_NUM);//אולטרסוניק
  xTaskCreatePinnedToCore(irObstacleTask, "IR_Task", 3000, NULL, 1, &TaskIRObstacle, APP_CPU_NUM);//IR חיישן מכשולים
  
  xTaskCreatePinnedToCore(dfPlayerTask, "DFPlayer_Task", 4000, NULL, 1, &TaskDFPlayer, APP_CPU_NUM);//רמקול MP3

  xTaskCreatePinnedToCore(
    decisionMakingTask,   
    "Decision_Task",      
    5000,                 
    NULL,                 
    3,                    
    NULL,                 
    APP_CPU_NUM           
  );
  
  Serial.println("--- כל התהליכונים נוצרו בהצלחה. ✅ ---"); 
  Serial.println("הפונקציה loop() תמשיך לרוץ במקביל או תישאר כמעט ריקה.");
}

// --- פונקציית LOOP ראשית (מינימלית) ---
void loop() {
  vTaskDelay(100 / portTICK_PERIOD_MS); 
}