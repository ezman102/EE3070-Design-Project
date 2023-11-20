#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_SSD1306.h>
#include <dht.h>
#include <ThingSpeak.h>
#include <WiFiEsp.h>
#include <Keypad.h>
#include <Adafruit_Fingerprint.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>


// Pin
// Temperateure & Humdity
#define DHT11_PIN 40
// Ultrasonic
#define trigPin 41
#define echoPin 2 //cannot be changed
// RFID
#define SS_PIN 53
#define RST_PIN 49
// LED
#define RED_LED_PIN 48
#define GREEN_LED_PIN 47
#define BLUE_LED_PIN 46
// Motor control pins
#define motorIN1Pin 45
#define motorIN2Pin 44
#define motorIN3Pin 43
#define motorIN4Pin 42
// Lightsensor
#define LightsensorPin A0
// Buzzer
#define buzzer 38 //buzzer to arduino pwn pin
// Fingerprint
SoftwareSerial mySerial(10, 11); // TX/RX on fingerprint sensor
// Keypad
byte pin_rows[4] = {13, 12, 9, 8};
byte pin_column[4] = {7, 6, 5, 4};

// Password
// Wifi password
char ssid[] = "EE3070_P1615_1";   //EE3070_P1615_1  TP-Link_002C
char pass[] = "EE3070P1615";     //EE3070P1615  20678203
// RFID password
byte storedUID[2][4] = {
  {0xF1, 0x3C, 0x82, 0x19},
  {0xBC, 0x74, 0x1A, 0x30}
};
// Keypad password
String password = "";
const String correctPassword = "12345678";
String masked_password = "";
// ThinkSpeak
unsigned long myChannelNumber = 2315925;
const char * myWriteAPIKey = "B66AQC1B5H7758EU";
const char * myReadAPIKey = "CGTJDYPH6BL5TVIP";
int statusCode = 0;
int uploadDebug = false;



//Setting
// Ultrasonic Setting
float uppestDistanceBound = 5; // larger than 5cm = the door opened
float lowestDistanceBound = 2.5;// smaller than 2.5cm = sensor error

// OLED Setting
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Wifi Setting
#define ESP_BAUDRATE 115200
WiFiEspClient client;

// DHT Setting
dht DHT;

// Buzzer Setting
bool alarmWarning = false; //buzzer warning when the door is opened abnormaly
const int tone_correct = 2000;
const int tone_wrong = 150;
volatile int tone_cur;
volatile int tone_type = 2500;

// Keypad Setting
volatile boolean PasswordState;
char keys[4][4] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, 4, 4);
char keyRead;

// Motor Setting
int mD1;
int mD2;
int mD3;
int mD4;
volatile int motorState; //1: motor is running 2:Open state 0:Close state
boolean enableLock = false;
boolean door_hasOpened = false;
int door_opened = 0;

// Light sensor Setting
int LightValue;
int LightWarningBound = 500; // Light > 500, warning

// Fingerprint Setting
volatile int finger_status = -1;
Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);

// RFID Setting
MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;

// Waiting Period Setting
const unsigned long UltrasonicCheckPeriod = 500;
const unsigned long FingerCheckPeriod = 500;
const unsigned long CloudWritePeriod = 100000;
const unsigned long CloudReadPeriod = 15000;
const unsigned long KeypadTimeoutPeriod = 10000;
const unsigned long LedScreenUpdatePeriod = 2000;
const unsigned long DoorClosePeriod = 200;
const unsigned long SleepPeriod = 20;
const unsigned long SleepEnablePeriod = 200;
//Variables
// Ultrasonic distance sensor Variables
volatile long duration;
volatile float distance;
volatile int d_flag;
// RunTime Variables
volatile unsigned long startTime;
volatile unsigned long PasswordTimerStart;
volatile unsigned long FingerTimerStart;
volatile unsigned long CloudTimerStart;
volatile unsigned long ModuleTimerStart;
volatile unsigned long TimeCur;
volatile unsigned long ReadCloudStart;
volatile unsigned long ReadCloudCur;
volatile unsigned long UltrasonicStart;
// Count Variables
int autolock_count = 0;
int upload_count = 0;
int sleep_count = 0;
int sleep_waitCount = 0;
volatile int toneCycle_count = 0;
// Timer Variables
volatile bool enableTimer1ISR;
volatile bool enableSleep;

void setup() {
  Serial.begin(115200);

  Serial1.begin(ESP_BAUDRATE);

  // Initialize ESP8266 module
  WiFi.init(&Serial1);

  // Connect to WiFi network
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print("Attempting to connect to SSID: ");
    Serial.println(ssid);
    WiFi.begin(ssid, pass);
    delay(5000);
  }
  Serial.println("Connected to WiFi");

  ThingSpeak.begin(client);  // Initialize ThingSpeak

  pinMode(buzzer, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT_PULLUP); // Sets the echoPin as an Input


  pinMode(motorIN1Pin, OUTPUT);
  pinMode(motorIN2Pin, OUTPUT);
  pinMode(motorIN3Pin, OUTPUT);
  pinMode(motorIN4Pin, OUTPUT);
  motorState = 0;
  //default in close state

  finger.begin(57600);
  if (finger.verifyPassword()) {
    Serial.println("Found fingerprint sensor!");
  } else {
    Serial.println("Did not find fingerprint sensor :(");
  }

  finger.getTemplateCount();
  Serial.print("Sensor contains "); Serial.print(finger.templateCount); Serial.println(" templates");
  Serial.println("Waiting for valid finger...");

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;) {} // Infinite loop to halt execution
  }
  display.display();
  delay(1000);
  display.clearDisplay();

  SPI.begin();
  rfid.PCD_Init();

  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);
  Serial.println(F("Waiting for card or password..."));
  displayMessage("Waiting for card or password...");
  TimerReset();
  attachInterrupt(digitalPinToInterrupt(echoPin), ultrasonicInterrupt, CHANGE);

  // Setup Timer 1
  cli();                            // Disable interrupts
  TCCR1A = 0;                       // Clear Timer 1 configuration
  TCCR1B = 0;
  TCNT1 = 0;                        // Reset Timer 1 counter
  OCR1A = 7812;                    // Set Timer 1 compare value for 1 second interrupt
  TCCR1B |= (1 << WGM12);           // Enable CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10);  // Set prescaler to 1024
  TIMSK1 |= (1 << OCIE1A);          // Enable compare match interrupt
  enableTimer1ISR = false;           // Disable timer1 interrupt
  // Setup sleep mode
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);  // Set sleep mode to power-down
  enableSleep = false;
  sei();                            // Enable interrupts
}



void loop() {
  if (not door_opened) {
    CheckRFID();
    CheckKeypad();
    CheckFingerprint();
    CheckCloud();
    sleep_count ++;
  }
  ultrasonicSensor();
  display_and_upload();
  if (enableLock && (distance > lowestDistanceBound) && (distance < uppestDistanceBound)) {
    autolock_count++;
    if (autolock_count >= (DoorClosePeriod / (1 + 9 * door_hasOpened))) { //avoid false trigger
      doorClose();
      autolock_count = 0;
    }
  }
  else if (door_opened && (distance > uppestDistanceBound)) {
    autolock_count = 0;
    //    enableLock = true;
    door_hasOpened = 1;
  }
  alarmsystem();

  if (enableSleep) {
    if (sleep_count < SleepEnablePeriod)
      enableSleep = false;
    else if (sleep_waitCount > SleepPeriod) {
      enterSleepMode();
      sleep_waitCount = 0;
    }
    else sleep_waitCount ++;
  } else if (sleep_count >= SleepEnablePeriod) {
    enableSleep = true;
    sleep_waitCount = 0;
  } else  enableSleep = false;

  delay(50);
}

void TimerReset() {
  FingerTimerStart = millis();
  CloudTimerStart = millis();
  ModuleTimerStart = millis();
  TimeCur = millis();
  ReadCloudStart = millis();
  UltrasonicStart = millis();
}

void CheckFingerprint() {
  if (door_opened)
    return;
  TimeCur = millis();
  if ((TimeCur - FingerTimerStart) > FingerCheckPeriod) {
    FingerTimerStart = millis();
    finger_status = getFingerprintIDez();
    if (finger_status != -1 and finger_status != -2) {
      displayMessage("Correct ID!");
      password = "";  // Reset the password string
      sleep_count = 0; // Reset sleep
      doorOpen();
    } else {
      if (finger_status == -2) {
        sleep_count = 0; // Reset sleep
        toneCycle_count = 1; // set Buzzer & LED run once
        tone_cur = tone_wrong; // set Buzzer & LED properties
        enableBuzzer(); // start Buzzer & LED
      }
    }
  }
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();

  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();

  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -2;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

void CheckKeypad() {
  if (door_opened)
    return;
  TimeCur = millis();
  keyRead = keypad.getKey();
  if (keyRead) {
    //reset sleep count
    sleep_count = 0;
    tone(buzzer, tone_type); // 1000 == Send 1KHz sound signal
    //    display.print("*");
    //    display.display();
    if (keyRead == '*') {
      PasswordState = false;
      Serial.println(F("\nClear!"));
      displayMessage("\nClear!");
      password = "";
      masked_password = "";
      delay(100);        // for 0.1 sec
      noTone(buzzer);
      return;
    }
    PasswordState = true;
    PasswordTimerStart = millis();
    password += keyRead;
    masked_password += '*';
    Serial.print(keyRead);
    delay(100);        // for 0.1 sec
    noTone(buzzer);
    if (password.length() == 8) {
      if (password == correctPassword) {
        Serial.println(F("\nCorrect Password!"));
        displayMessage("Correct Password!");
        doorOpen();
      }
      else {
        Serial.println(F("\nWrong Password!"));
        displayMessage("Wrong Password!");
        password = "";  // Reset the password string
        masked_password = "";


        toneCycle_count = 3; // set Buzzer & LED run time
        tone_cur = tone_wrong; // set Buzzer & LED properties
        enableBuzzer(); // start Buzzer & LED
        delay(2000);
      }
    }
  } else if (((TimeCur - PasswordTimerStart) >= KeypadTimeoutPeriod) && PasswordState) {
    PasswordState = false;
    Serial.println(F("\nTime out!"));
    Serial.println(F("\nPlease input password again!"));
    displayMessage("Time out!\nPlease input password again!");
    password = "";
    masked_password = "";
  }
}

void CheckRFID() {
  if (door_opened)
    return;
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    sleep_count = 0; // Reset sleep
    if (isUIDMatch(rfid.uid.uidByte, storedUID, sizeof(storedUID) / sizeof(storedUID[0]))) {
      Serial.println("Correct Card!");
      displayMessage("Correct Card!");
      doorOpen();
    }
    else {
      Serial.println("Wrong Card!");
      displayMessage("Wrong Card!");
      toneCycle_count = 3; // set Buzzer & LED run time
      tone_cur = tone_wrong; // set Buzzer & LED properties
      enableBuzzer(); // start Buzzer & LED
    }
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
}

void CheckCloud() {
  if ((door_opened) || PasswordState)
    return;
  ReadCloudCur = millis();
  if (ReadCloudCur - ReadCloudStart >= CloudReadPeriod) {
    ReadCloudStart = millis();
    statusCode = ThingSpeak.readMultipleFields(myChannelNumber);
    int open_request = ThingSpeak.readIntField(myChannelNumber, 6, myReadAPIKey);
    String createdAt = ThingSpeak.getCreatedAt();
    if (ThingSpeak.getLastReadStatus() == 200) { // Check if the read was successful
      Serial.print("Latest Value in Field 6: ");
      Serial.println(open_request);
    } else {
      Serial.println("Failed to read from ThingSpeak");
    }
    Serial.print("Time consume: ");
    Serial.println(millis() - ReadCloudCur);
    if (open_request == 1) {
      ThingSpeak.setField(6, 0);
      doorOpen();

    }

  }
}

bool isUIDMatch(byte * a, byte storedUID[][4], int numUIDs) {
  for (int i = 0; i < numUIDs; i++) {
    bool match = true;
    for (byte j = 0; j < 4; j++) {
      if (a[j] != storedUID[i][j]) {
        match = false;
        break;
      }
    }
    if (match)
      return true;
  }
  return false;
}

void setColor(int red, int green, int blue) {
  analogWrite(RED_LED_PIN, red);
  analogWrite(GREEN_LED_PIN, green);
  analogWrite(BLUE_LED_PIN, blue);
}

void displayMessage(const char* msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(msg);
  display.display();
}


// Angle: 0-360, Direction: 0(anticlock), 1(clockwise), Speed: 0-100
void MotorRun(int angle, int dirct, int speed) {
  motorState = 1; //state the motor is running
  int step = 0;
  int runTimes = 5.688 * angle;
  if (dirct == 0) {
    while (runTimes--) {
      switch (step) {
        case 0: mD1 = 1; mD2 = 1; mD3 = 0; mD4 = 0; break;
        case 1: mD1 = 0; mD2 = 1; mD3 = 1; mD4 = 0; break;
        case 2: mD1 = 0; mD2 = 0; mD3 = 1; mD4 = 1; break;
        case 3: mD1 = 1; mD2 = 0; mD3 = 0; mD4 = 1; break;
      };
      MotorIN(mD1, mD2, mD3, mD4);
      if (step >= 3) step = 0;
      else step++;
      if (speed > 100) speed = 100;
      delay(104 - speed);
      MotorIN(0, 0, 0, 0);
    }
    motorState = 2; //state motor stop in open state

  } else {
    setColor(0, 255, 0);  // Turn Blue
    while (runTimes--) {
      switch (step) {
        case 0: mD1 = 0; mD2 = 0; mD3 = 1; mD4 = 1; break;
        case 1: mD1 = 0; mD2 = 1; mD3 = 1; mD4 = 0; break;
        case 2: mD1 = 1; mD2 = 1; mD3 = 0; mD4 = 0; break;
        case 3: mD1 = 1; mD2 = 0; mD3 = 0; mD4 = 1; break;
      };
      MotorIN(mD1, mD2, mD3, mD4);
      if (step >= 3) step = 0;
      else step++;
      if (speed > 100) speed = 100;
      delay(104 - speed);
      MotorIN(0, 0, 0, 0);
    }
    motorState = 0; //state motor stop in close state
    setColor(0, 0, 0);  // Clear
  }
  mD1 = 0; mD2 = 0; mD3 = 0; mD4 = 0;

}

// Send signals to motor
void MotorIN(int D1, int D2, int D3, int D4) {
  digitalWrite(motorIN1Pin, D1);
  digitalWrite(motorIN2Pin, D2);
  digitalWrite(motorIN3Pin, D3);
  digitalWrite(motorIN4Pin, D4);
}


void doorClose() {

  MotorRun(90, 1, 100); // Automatic door locking
  door_opened = false;
  door_hasOpened = 0;
  enableLock = false;
  Serial.println("Locked");
  toneCycle_count = 1; // set Buzzer & LED run once
  tone_cur = tone_wrong; // set Buzzer & LED properties
  enableBuzzer(); // start Buzzer & LED
  ThingSpeak.setField(5, 0);
  int result = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  Serial.print("Upload Result: ");
  Serial.println(result);  // Print result for debugging
  TimerReset();
  sleep_count = 0;
}
void doorOpen() {
  sleep_count = 0; // Reset sleep
  password = "";  // Reset the password string
  masked_password = "";
  PasswordState = false;
  //reset sleep count
  sleep_count = 0;

  setColor(0, 255, 0);  // Green

  MotorRun(90, 0, 100); // Turn anticlockwise 90 degrees
  door_opened = true;
  //enableLock = false;
  Serial.println("Opened");
  toneCycle_count = 1; // set Buzzer & LED run once
  tone_cur = tone_correct; // set Buzzer & LED properties
  enableBuzzer(); // start Buzzer & LED
  ThingSpeak.setField(5, 1);
  int result = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  Serial.print("Upload Result: ");
  Serial.println(result);  // Print result for debugging
  enableLock = true;
  TimerReset();
}

void alarmsystem() {
  if ((not door_opened) && ((distance >= uppestDistanceBound) || (LightValue > LightWarningBound))) {
    alarmWarning = true;
    //reset sleep count
    sleep_count = 0;
    if (tone_cur == 0) {
      toneCycle_count = 999; // set Buzzer & LED run once
      tone_cur = tone_wrong; // set Buzzer & LED properties
      enableBuzzer(); // start Buzzer & LED
    }
  }
  else {
    alarmWarning = false;
    // disableBuzzer();
    toneCycle_count = 0;
  }
}
//UltrasonicCheckPeriod
void ultrasonicSensor() {
  if (millis() - UltrasonicStart >= UltrasonicCheckPeriod) {
    UltrasonicStart = millis();
    sendPulse();
    d_flag = 1;
  }
}
void calculateDistance() {
  duration = TimeCur - startTime;
  // Calculating the distance
  distance = duration * 0.034 / 2;
  if (distance > 500)
    distance = -1;
  if (distance > 300)
    distance = 300;
  d_flag = 0;
}




void sendPulse() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}

void ultrasonicInterrupt() {
  if (true && (motorState != 1)) {
    if (d_flag == 1) {
      startTime = micros();
      d_flag = 2;
    }
    else if (d_flag == 2) {
      TimeCur = micros();
      calculateDistance();
    }
  }
}

//void uploadALL() {
//  while (result != 200) {
//    int chk = DHT.read11(DHT11_PIN);
//    int humidity_value = int(DHT.humidity);
//    int temperature_value = int(DHT.temperature);
//    LightValue = analogRead(LightsensorPin);
//    ThingSpeak.setField(1, humidity_value);
//    ThingSpeak.setField(2, temperature_value);
//    ThingSpeak.setField(3, LightValue);
//    ThingSpeak.setField(4, distance);
//    if (door_opened)
//      ThingSpeak.setField(5, 1);
//    else
//      ThingSpeak.setField(5, 0);
//    int result = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
//    Serial.print("Upload Result: ");
//    Serial.println(result);  // Print result for debugging
//  }
//}

void display_and_upload() {

  TimeCur = millis();
  if ((TimeCur - ModuleTimerStart) > LedScreenUpdatePeriod) {
    int chk = DHT.read11(DHT11_PIN);
    LightValue = analogRead(LightsensorPin);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.print("Humidity:    ");
    display.println(DHT.humidity, 1);
    display.print("Temperature: ");
    display.println(DHT.temperature, 1);
    display.print("Light:       ");
    display.println(LightValue);
    display.print("Distance:    ");
    display.println(distance);
    display.print("Door locked:    ");
    display.println(door_opened);
    display.print("Password:  ");
    display.print(masked_password);
    display.println();
    display.display();
    ModuleTimerStart = millis();
  }
  TimeCur = millis();
  if ((TimeCur - CloudTimerStart >= CloudWritePeriod) && (alarmWarning == false) && (!PasswordState)) {
    int humidity_value = int(DHT.humidity);
    int temperature_value = int(DHT.temperature);
    if (upload_count == 0) {
      ThingSpeak.setField(1, humidity_value);
    }
    if (upload_count == 1) {
      ThingSpeak.setField(2, temperature_value);
    }
    if (upload_count == 2) {
      ThingSpeak.setField(3, LightValue);
    }
    if (upload_count == 3) {
      ThingSpeak.setField(4, distance);
      upload_count = -1;
    }
    int result = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    Serial.print("Upload Result: ");
    Serial.println(result);  // Print result for debugging
    Serial.print("Time consume: ");
    Serial.println(millis() - TimeCur);
    CloudTimerStart = millis();
    upload_count++;
  }
}

void enableBuzzer() {
  TCNT1 = 0;
  TIMSK1 |= (1 << OCIE1A);          // Enable timer interrupt
}

void disableBuzzer() {
  noTone(buzzer);     // Turn off the buzzer
  tone_cur = 0;

  TIMSK1 &= ~(1 << OCIE1A);         // Disable timer interrupt
}
//
//void enterSleepMode() {
//  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//  sleep_enable();
//  // enter sleep
//  sleep_mode();
//  // wake up
//  sleep_disable();
//}

// Timer1 ISR
ISR(TIMER1_COMPA_vect) {
  if ((toneCycle_count % 2 == 1) && (toneCycle_count >= 0)) {
    tone(buzzer, tone_cur);
    switch (tone_cur) {
      case tone_correct:
        break;
      case tone_wrong:
        setColor(255, 0, 0);  // Red
        break;
      default:
        setColor(0, 0, 0);  // Clear
        break;
    }
    toneCycle_count --;
  } else {
    noTone(buzzer);
    switch (tone_cur) {
      case tone_correct:
        break;
      case tone_wrong:
        setColor(0, 0, 0);  //
        break;
      default:
        setColor(0, 0, 0);  // Clear
        break;
    }
    toneCycle_count --;
    if (toneCycle_count < 0)
      disableBuzzer();
  }
}

ISR(WDT_vect)
{
  Serial.println(".");
}

void enterSleepMode()
{
  Serial.println("sleep");
  // Configure the watchdog timer
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
   MCUSR &= ~(1 << WDRF);  // Clear the Watchdog System Reset Flag
  WDTCSR |= (1 << WDCE) | (1 << WDE);  // Enable Watchdog Timer Configuration
  WDTCSR = (1 << WDP2) | (1 << WDP1) | (1 << WDP0);  // Set Watchdog Timer Prescaler to 4 seconds
  WDTCSR |= _BV(WDIE); 
  delay(1);
  // Enter power-down sleep mode
  sleep_mode();
  delay(1);
  wdt_disable();
  Serial.println("wake up");
}
