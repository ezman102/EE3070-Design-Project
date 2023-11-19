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
byte pin_column[4] = {7, 6, 5, 3};
#define keypadIntPin 3
