#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pins
#define ENC 8   // Optical Encoder with 30 pulses/rev
#define EN 9    // Pin to enable motor driver (pwm) !!! NOT 2, because this pin has no pwm !!!
#define PH 3    // Pin to set direction
#define POT A0  // Abnehmer des Potentiometers

#define MAX_RPM 1500 // maximale Drehzahl

// Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// put function declarations here:
void setMotor(int dir, int pwmVal, int pwm, int phase);
void readEncoder();

// globals
long prevT = 0;
int posPrev = 0;

// volatile for variables used in an interrupt
volatile int pos_i = 0;

float vFilt = 0;
float vPrev = 0;

float eintegral = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENC, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(POT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC), readEncoder, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  
  display.display();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {
  int pos = 0;
  // ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
  //   pos = pos_i;
  // }

  pos = pos_i;
  
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / 1.0e6;
  float velocity = (pos - posPrev) / deltaT;
  posPrev = pos;
  prevT = currT;

  // Convert counts/s to RPM
  float v = velocity / 60.0 * 60.0;

  Serial.print(pos);
  Serial.print("\t");
  Serial.print(v);
  Serial.print("\t");

  // Low-pass filter (25 Hz cutoff)
  vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
  vPrev = v;

  // Set target velocity
  //float vt = 100*(sin(currT/1e6)>0);
  int pot = analogRead(POT);
  float vt = map(pot, 0, 1025, 200, MAX_RPM);

  // Compute control signal u
  float kp = 0.06;
  float ki = 0.03;
  float e = vt - vFilt;
  eintegral = eintegral + e*deltaT;

  float u = kp*e + ki*eintegral;

  // Set motor speed and direction
  int dir = 1;
  if (u<0) {
    dir = -1;
  }
  int pwr = (int) fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }
  setMotor(dir, pwr, EN, PH);

  Serial.print(vt);
  Serial.print("\t");
  Serial.println(vFilt);

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(80, 16);
  display.print(vt, 0);
  display.setCursor(0, 0);
  display.setTextSize(3);
  display.print(vFilt, 0);
  display.display();

  delay(25);
}

void setMotor(int dir, int pwmVal, int pwm, int phase) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(PH, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(PH, LOW);
  }
}

void readEncoder() {
  pos_i++;
}