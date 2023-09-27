#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Arduino.h>
#include <U8x8lib.h>
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif
//L289N
#define enA 5
#define in1 6
#define in2 7
#define enB 3
#define in3 4
#define in4 2
//NRF24L01
//9 CE, 10 CSN, 11 MISO, 12 MOSI, 13 SCK
RF24 radio(9, 10);
//LCD
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x8(/* reset=*/U8X8_PIN_NONE);
//BUZZER
//A0
//IMPACT
//A1
//VOLT
//A2
//WEAPON MOSFET
#define weapon 1
//WEAPON Lights
#define lights 0

const byte addresses[][6] = { "00001", "00002", "00003", "00004" };
int joystick[5];
int stat[3];
int button1State = HIGH;      // Current state of the button
int lastButton1State = HIGH;  // Previous state of the button
int button2State = HIGH;      // Current state of the button
int lastButton2State = HIGH;  // Previous state of the button
bool isWeaponOn = false;      // Flag to track Weapon state
bool isLightsOn = false;      // Flag to track Weapon state
enum MotorState {
  STOP,
  FWD,
  RWD
};

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(addresses[0]);     //Setting   the address at which we will send the data
  radio.openReadingPipe(1, addresses[1]);  //Setting the address at which we will receive the data
  radio.setPALevel(RF24_PA_MAX);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  u8x8.begin();
}
void pre(void) {
  u8x8.clear();
  u8x8.setCursor(0, 1);
  u8x8.print("  BATTLE BOT ");
}
void controlMotor(int enPin, int in1Pin, int in2Pin, int speed, MotorState state) {
  analogWrite(enPin, speed);
  digitalWrite(in1Pin, state == FWD ? LOW : HIGH);
  digitalWrite(in2Pin, state == RWD ? LOW : HIGH);
}

void loop() {
  radio.startListening();
  delay(50);

  if (radio.available()) {
    radio.read(&joystick, sizeof(joystick));

    controlMotor(enA, in1, in2, abs(joystick[0]), joystick[0] > 0 ? FWD : (joystick[0] < 0 ? RWD : STOP));
    controlMotor(enB, in3, in4, abs(joystick[2]), joystick[2] > 0 ? FWD : (joystick[2] < 0 ? RWD : STOP));
    button1State = joystick[1];
    button2State = joystick[3];
    if (button1State != lastButton1State) {
      // If the button is pressed (LOW)
      if (button1State == LOW) {
        isWeaponOn = !isWeaponOn;                       // Toggle the Weapon state
        digitalWrite(weapon, isWeaponOn ? HIGH : LOW);  // Turn the Weapon on/off
      }
      lastButton1State = button1State;
    }

    if (button2State != lastButton2State) {
      // If the button is pressed (LOW)
      if (button2State == LOW) {
        isLightsOn = !isLightsOn;                       // Toggle the Lights state
        digitalWrite(weapon, isLightsOn ? HIGH : LOW);  // Turn the Lights on/off
      }
      lastButton2State = button2State;
    }
    Serial.print("A: ");
    Serial.print(joystick[0] > 0 ? "FWD  " : (joystick[0] < 0 ? "RWD  " : "STOP "));
    Serial.print(abs(joystick[0]));
    Serial.print("% WEAPON: ");
    Serial.println(isWeaponOn ? "OFF" : "ON");

    Serial.print("B: ");
    Serial.print(joystick[2] > 0 ? "FWD  " : (joystick[2] < 0 ? "RWD  " : "STOP "));
    Serial.print(abs(joystick[2]));
    Serial.print("% LIGHTS: ");
    Serial.println(isLightsOn ? "OFF" : "ON");
    int i;
    uint8_t c, r, d;
    pre();
    u8x8.setCursor(0, 2);
    u8x8.print(joystick[0] > 0 ? "A: FWD  " : (joystick[0] < 0 ? "A: RWD  " : "A: STOP "));
    u8x8.print(abs(joystick[0]));
    u8x8.setCursor(0, 3);
    u8x8.print(joystick[2] > 0 ? "B: FWD  " : (joystick[2] < 0 ? "B: RWD  " : "B: STOP "));
    u8x8.print(abs(joystick[2]));
    u8x8.setFont(u8x8_font_amstrad_cpc_extended_f);
    u8x8.setCursor(0, 4);
    u8x8.print("WEAPON: ");
    u8x8.print(isWeaponOn ? "OFF" : "ON");
    u8x8.setCursor(0, 5);
    u8x8.print("LIGHTS: ");
    u8x8.print(isLightsOn ? "OFF" : "ON");
  }

  radio.stopListening();

  stat[0] = random(1, 5);  //power
  stat[1] = random(1, 5);  //health

  radio.write(stat, sizeof(stat));
  Serial.print("PWR: ");
  Serial.print(stat[0]);
  Serial.print("/5 HP: ");
  Serial.print(stat[1]);
  Serial.println("/5");
  u8x8.setCursor(0, 6);
  u8x8.print("PWR:    ");
  u8x8.print(stat[0]);
  u8x8.setCursor(0, 7);
  u8x8.print("HP:     ");
  u8x8.print(stat[1]);

  radio.startListening();
  delay(300);
}