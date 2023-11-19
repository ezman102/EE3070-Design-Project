#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_SSD1306.h>
#include <dht.h>
#include <ThingSpeak.h>
#include <WiFiEsp.h>
#include <Keypad.h>
#include <Adafruit_Fingerprint.h>

#define ESP_BAUDRATE 115200
char ssid[] = "TP-Link_002C";
char pass[] = "20678203";
WiFiEspClient client;
unsigned long myChannelNumber = 2315925;
const char * myWriteAPIKey = "B66AQC1B5H7758EU";

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


#define DHT11_PIN 40
dht DHT;
#define trigPin 41
#define echoPin 2 //cannot be changed
volatile long duration;
volatile float distance;
volatile int d_flag;

volatile unsigned long startTime;
//volatile unsigned long endTime;

volatile unsigned long PasswordTimerStart;
volatile boolean PasswordState;

volatile unsigned long FingerTimerStart;

volatile unsigned long CloudTimerStart;
//volatile unsigned long CloudTimerCur;

volatile unsigned long ModuleTimerStart;
//volatile unsigned long ModuleTimerCur;

volatile unsigned long TimeCur;

const int ROW_NUM = 4;
const int COLUMN_NUM = 4;
char keys[ROW_NUM][COLUMN_NUM] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte pin_rows[ROW_NUM] = {13, 12, 9, 8};
byte pin_column[COLUMN_NUM] = {7, 6, 5, 4};
Keypad keypad = Keypad(makeKeymap(keys), pin_rows, pin_column, ROW_NUM, COLUMN_NUM);

#define SS_PIN 53
#define RST_PIN 49
#define RED_LED_PIN 48
#define GREEN_LED_PIN 47
#define BLUE_LED_PIN 46

MFRC522 rfid(SS_PIN, RST_PIN);
MFRC522::MIFARE_Key key;
String password = "";
const String correctPassword = "12345678";

byte storedUID[2][4] = {
  {0xF1, 0x3C, 0x82, 0x19},
  {0xBC, 0x74, 0x1A, 0x30}
};

// Motor control pins
#define motorIN1Pin 45
#define motorIN2Pin 44
#define motorIN3Pin 43
#define motorIN4Pin 42
int mD1;
int mD2;
int mD3;
int mD4;
volatile int motorState;
float uppestDistanceBound = 4;
float lowestDistanceBound = 2.5;


#define LightsensorPin A0
#define buzzer 38 //buzzer to arduino pwn pin 12
volatile boolean enableLock = false;
bool door_opened = false;
bool alarmWarning = false; //buzzer warning when the door is opened abnormaly
int LightValue;
int LightWarningBound = 500;
char keyRead;

volatile int finger_status = -1;
SoftwareSerial mySerial(10, 11); // TX/RX on fingerprint sensor

Adafruit_Fingerprint finger = Adafruit_Fingerprint(&mySerial);



void setup() {
  Serial.begin(115200);

  //  Serial1.begin(ESP_BAUDRATE);
  //
  //  // Initialize ESP8266 module
  //  WiFi.init(&Serial1);
  //
  //  // Connect to WiFi network
  //  while (WiFi.status() != WL_CONNECTED) {
  //    Serial.print("Attempting to connect to SSID: ");
  //    Serial.println(ssid);
  //    WiFi.begin(ssid, pass);
  //    delay(5000);
  //  }
  //  Serial.println("Connected to WiFi");
  //
  //  ThingSpeak.begin(client);  // Initialize ThingSpeak

  pinMode(buzzer, OUTPUT);
  pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin, INPUT_PULLUP); // Sets the echoPin as an Input
  //attachInterrupt(digitalPinToInterrupt(2), echo_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(echoPin), interruptLogic, CHANGE);
  pinMode(motorIN1Pin, OUTPUT);
  pinMode(motorIN2Pin, OUTPUT);
  pinMode(motorIN3Pin, OUTPUT);
  pinMode(motorIN4Pin, OUTPUT);
  motorState = 0;

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

  FingerTimerStart = millis();
  CloudTimerStart = millis();
  ModuleTimerStart = millis();
  TimeCur = millis();
}



void loop() {
  if (not door_opened) {
    CheckRFID();
    CheckKeypad();
    CheckFingerprint();
  }
  ultrasonicSensor();
  display_and_upload_All();
  if (enableLock && (distance > lowestDistanceBound) && (distance < uppestDistanceBound))
    doorClose();
  else if (distance > uppestDistanceBound)
    enableLock = true;
  alarmsystem();
  delay(10);
}

void CheckFingerprint() {
  TimeCur = millis();
  if ((TimeCur - FingerTimerStart) > 500) {
    FingerTimerStart = millis();
    finger_status = getFingerprintIDez();
    if (finger_status != -1 and finger_status != -2) {
      displayMessage("Correct ID!");
      password = "";  // Reset the password string
      doorOpen();
    } else {
      if (finger_status == -2) {
        setColor(255, 0, 0);  // Red
        delay(1000);  // Keep the LED on for 1 second
        setColor(0, 0, 0);  // Turn off LED
      }
    }
  }
}

// returns -1 if failed, otherwise returns ID #
int getFingerprintIDez() {
  uint8_t p = finger.getImage();
  if (p != 2) {
    Serial.println(p);
  }
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.image2Tz();
  if (p != 2) {
    Serial.println('.');
  }
  if (p != FINGERPRINT_OK)  return -1;

  p = finger.fingerFastSearch();
  if (p != FINGERPRINT_OK)  return -2;

  // found a match!
  Serial.print("Found ID #"); Serial.print(finger.fingerID);
  Serial.print(" with confidence of "); Serial.println(finger.confidence);
  return finger.fingerID;
}

void CheckKeypad() {
  TimeCur = millis();
  keyRead = keypad.getKey();
  if (keyRead) {
    if (keyRead == '*') {
      PasswordState = false;
      Serial.println(F("\nClear!"));
      displayMessage("\nClear!");
      password = "";
      return;
    }
    PasswordState = true;
    PasswordTimerStart = millis();
    password += keyRead;
    Serial.print(keyRead);

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
        setColor(255, 0, 0);  // Red
        delay(1000);  // Keep the LED on for 1 second
        setColor(0, 0, 0);  // Turn off LED
      }
    }
  } else if (((TimeCur - PasswordTimerStart) >= 10000) && PasswordState) {
    PasswordState = false;
    Serial.println(F("\nTime out!"));
    Serial.println(F("\nPlease input password again!"));
    displayMessage("Time out!\nPlease input password again!");
    password = "";
  }
}

void CheckRFID() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    if (isUIDMatch(rfid.uid.uidByte, storedUID, sizeof(storedUID) / sizeof(storedUID[0]))) {
      Serial.println("Correct Card!");
      displayMessage("Correct Card!");
      doorOpen();
    }
    else {
      Serial.println("Wrong Card!");
      displayMessage("Wrong Card!");
      setColor(255, 0, 0);  // Red
      delay(1000);  // Keep the LED on for 1 second
      setColor(0, 0, 0);  // Turn off LED
    }
    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();
  }
}

bool isUIDMatch(byte *a, byte storedUID[][4], int numUIDs) {
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

void display_and_upload_All() {

  TimeCur = millis();
  if ((TimeCur - ModuleTimerStart) > 2000) {
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
    display.display();
    int humidity_value = int(DHT.humidity);
    int temperature_value = int(DHT.temperature);

    ModuleTimerStart = millis();
  }
  //  TimeCur = millis();
  //  if ((TimeCur - CloudTimerStart >= 20000) && (alarmWarning == false)) {
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
  //    CloudTimerStart = millis();
  //  }
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
    mD1 = 0; mD2 = 0; mD3 = 0; mD4 = 0;
  } else {
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
    mD1 = 0; mD2 = 0; mD3 = 0; mD4 = 0;
  }
}

// Send signals to motor
void MotorIN(int D1, int D2, int D3, int D4) {
  digitalWrite(motorIN1Pin, D1);
  digitalWrite(motorIN2Pin, D2);
  digitalWrite(motorIN3Pin, D3);
  digitalWrite(motorIN4Pin, D4);
}



void simpleUltrasonic() {
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2;
}

void doorClose() {
  setColor(0, 0, 0);  // Turn off LED
  MotorRun(90, 1, 100); // Automatic door locking
  door_opened = false;
  enableLock = false;
  Serial.println("Locked");
  tone(buzzer, 7500); // 1000 == Send 1KHz sound signal
  delay(500);        // for 0.5 sec
  noTone(buzzer);
}
void doorOpen() {
          password = "";  // Reset the password string
          PasswordState = false;
  setColor(0, 255, 0);  // Green
  MotorRun(90, 0, 100); // Turn anticlockwise 90 degrees
  door_opened = true;
  enableLock = false;
  Serial.println("Opened");
}

void alarmsystem() {
  if ((not door_opened) && ((distance >= uppestDistanceBound) || (LightValue > LightWarningBound))) {
    alarmWarning = true;
    tone(buzzer, 7500); // 1000 == Send 1KHz sound signal
    setColor(255, 0, 0);  // Red
    delay(500);        // for 0.5 sec
    noTone(buzzer);
    setColor(0, 0, 0);  // Turn off LED
    if (motorState == 0) {
      MotorRun(90, 0, 100); // Turn anticlockwise 90 degrees
      enableLock = true;
    }
    else
      delay(500);
  }
  else alarmWarning = false;
}

void ultrasonicSensor() {
  sendPulse();
  d_flag = 1;
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

void interruptLogic() {
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
