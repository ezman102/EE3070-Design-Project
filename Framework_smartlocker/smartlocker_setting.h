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
