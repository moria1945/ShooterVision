// קוד בדיקה ייעודי להפעלת DFPlayer Mini עם ESP32.
// קוד זה מכיל רק את הפונקציונליות ההכרחית להפעלת ה-DFPlayer Mini.

#include <HardwareSerial.h>      // ספרייה לתקשורת טורית חומרתית ב-ESP32
#include <DFRobotDFPlayerMini.h> // ספרייה לשליטה ב-DFPlayer Mini

// הגדרת פיני ה-UART עבור ה-DFPlayer Mini ב-ESP32.
// ודא חיווט נכון:
// ESP32 GPIO2 (TX) מתחבר לפין RX של ה-DFPlayer Mini.
// ESP32 GPIO4 (RX) מתחבר לפין TX של ה-DFPlayer Mini.
// (שים לב: במודול DFPlayer Mini, ה-RX הוא הקלט וה-TX הוא הפלט שלו).
#define DFPLAYER_TX_PIN 16 // פין TX של ESP32 (פלט)
#define DFPLAYER_RX_PIN 17 // פין RX של ESP32 (קלט)

// יצירת אובייקט HardwareSerial. נשתמש ב-UART2 עבור פינים GPIO2 ו-GPIO4.
// ב-ESP32, יציאה טורית 2 (UART2) משתמשת ב-GPIO17 (TX) ו-GPIO16 (RX) כברירת מחדל,
// אך ניתן לשנות זאת בקלות לפנים אחרים כמו 2 ו-4.
HardwareSerial myDFPlayerSerial(2); // יוצרים אובייקט עבור UART2

// יצירת אובייקט DFRobotDFPlayerMini.
DFRobotDFPlayerMini myDFPlayer;

void setup() {
  // אתחול תקשורת Serial Monitor לצורך הדפסת הודעות דיבוג.
  // ודא שקצב הבאוד ב-Serial Monitor מוגדר ל-115200.
  Serial.begin(9600); 
  Serial.println("--- מתחיל בדיקת DFPlayer Mini ---");

  // אתחול התקשורת הטורית החומרתית (UART2) עם ה-DFPlayer Mini.
  // קצב הבאוד ל-DFPlayer הוא בדרך כלל 9600.
  // SERIAL_8N1 הוא פורמט ברירת המחדל של נתונים (8 ביטים, ללא זוגיות, 1 סטופ ביט).
  // הפרמטרים האחרונים מגדירים את פיני ה-RX וה-TX עבור ה-UART2 באופן מפורש.
  myDFPlayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
  
  Serial.println("מנסה לאתחל DFPlayer Mini...");

  // ניסיון לאתחל את ה-DFPlayer Mini.
  // אם .begin() מחזיר false, משהו לא עובד.
  while (!myDFPlayer.begin(myDFPlayerSerial)) {
    Serial.println("שגיאה: DFPlayer Mini לא אותחל!");
    Serial.println("בדוק את הדברים הבאים:");
    Serial.println("1. חיווט: ודא ש-TX של ESP32 (GPIO2) מחובר ל-RX של DFPlayer, ו-RX של ESP32 (GPIO4) מחובר ל-TX של DFPlayer.");
    Serial.println("2. מתח: ודא ש-DFPlayer מקבל 5V יציבים ושיש חיבור אדמה משותף עם ה-ESP32.");
    Serial.println("3. כרטיס SD: ודא שכרטיס MicroSD בפורמט FAT32, מוכנס היטב, ויש עליו תיקיית 'mp3'.");
    Serial.println("4. קבצי אודיו: ודא שיש קבצי MP3 בתיקיית 'mp3' בשמות כמו 0001.mp3, 0002.mp3, וכו'.");
    // עצירה כאן אם האתחול נכשל, כדי לא לנסות לנגן.
    delay(1000);
  }

  Serial.println("DFPlayer Mini אותחל בהצלחה!");
  
  // הגדרת עוצמת השמע. ערך בין 0 ל-30.
  myDFPlayer.volume(30); // קובע את העוצמה ל-30 (מקסימום).
  Serial.print("עוצמת שמע הוגדרה ל: ");

  // --- בדיקת נגינה: נגן קובץ 0001.mp3 ---
  Serial.println("מבצע בדיקת נגינה של קובץ 0001.mp3...");
  myDFPlayer.play(1); // נגן את קובץ מספר 1
  delay(3000); // המתן 3 שניות כדי לתת לקובץ להתנגן

  // נגן קובץ 2
  Serial.println("מבצע בדיקת נגינה של קובץ 0002.mp3...");
  myDFPlayer.play(2); // נגן את קובץ מספר 2
  delay(3000); // המתן 3 שניות כדי לתת לקובץ להתנגן
  
  Serial.println("--- בדיקת DFPlayer Mini הסתיימה ב-setup ---");
}

void loop() {

  myDFPlayer.play(1); // נגן את קובץ מספר 1
  // ב-loop(), ניתן להוסיף לוגיקה נוספת לנגינה.
  // myDFPlayer.loop(); // אם יש צורך בטיפול בפקודות נכנסות מה-DFPlayer (פחות נפוץ)
}
