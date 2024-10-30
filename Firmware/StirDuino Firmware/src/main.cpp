#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pins
#define ENC 8   // Optical Encoder with 30 pulses/rev
#define EN 9    // Pin to enable motor driver (pwm) !!! NOT 2, because this pin has no pwm !!!
#define PH 3    // Pin to set direction
#define POT A0  // Abnehmer des Potentiometers

#define MAX_RPM 1400  // maximale Drehzahl. Bei zu hoher Drehzahl können die Encoder-Ticks nicht mehr registriert werden.
#define MIN_RPM 0     // minimale Drehzahl. Zu niedrige Drehzahlen können instabil sein.
#define CONTROLLER_REFRESH_RATE 100 // Hz

// Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define DISPLAY_REFRESH_RATE 8 // refreshs per second

// put function declarations here:
void setMotor(int dir, int pwmVal, int pwm, int phase);
void readEncoder();

// globals
int pos = 0;
float v = 0;
int pwr = 0;

long prevT = 0;
int posPrev = 0;

// volatile for variables used in an interrupt
volatile int pos_i = 0;

float vt = 0;
float vFilt = 0;
float vPrev = 0;

float eintegral = 0;

const int avrgCount = 64;

float vtAvrgBuffer[avrgCount];
int vtNextAvrg = 0;
float vtAvrg = 0;

float vFiltAvrgBuffer[avrgCount];
int vFiltNextAvrg = 0;
float vFiltAvrg = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(ENC, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(POT, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC), readEncoder, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  
  display.setRotation(2);
  display.display();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);
}

void loop() {

  for (int i = 0; i <= (CONTROLLER_REFRESH_RATE / DISPLAY_REFRESH_RATE); i++) {
    pos = 0;
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
    v = velocity / 60.0 * 60.0;

    // Low-pass filter (25 Hz cutoff)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
    vPrev = v;

    vFiltAvrgBuffer[vFiltNextAvrg++] = vFilt;
    if (vFiltNextAvrg >= avrgCount) {
      vFiltNextAvrg = 0;
    }
    vFiltAvrg = 0;
    for (int i = 0; i < avrgCount; i++) {
      vFiltAvrg += vFiltAvrgBuffer[i];
    }
    vFiltAvrg /= avrgCount;

    // Set target velocity
    //float vt = 100*(sin(currT/1e6)>0);
    int pot = analogRead(POT);
    vt = map(pot, 0, 1023, MIN_RPM, MAX_RPM + 4);
    if (vt < 10) {
      vt = 0;
    }

    vtAvrgBuffer[vtNextAvrg++] = vt;
    if (vtNextAvrg >= avrgCount) {
      vtNextAvrg = 0;
    }
    vtAvrg = 0;
    for (int i = 0; i < avrgCount; i++) {
      vtAvrg += vtAvrgBuffer[i];
    }
    vtAvrg /= avrgCount;

    // Compute control signal u
    float kp = 0.5;
    float ki = 0.2;
    float e = vt - vFilt;
    eintegral = eintegral + e*deltaT;

    float u = kp*e + ki*eintegral;

    // Set motor speed and direction
    int dir = 1;
    if (u<0) {
      dir = -1;
    }
    pwr = (int) fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }
    if (vt < 10) {
      pwr = 0;
    }
    setMotor(dir, pwr, EN, PH);

    delay(1000 / CONTROLLER_REFRESH_RATE); // Update values every 10 ms
  }
  
  display.clearDisplay();
  display.setCursor(80, 2);
  display.setTextSize(1);
  display.print("SET");
  display.setCursor(80, 12);
  display.setTextSize(2);
  display.print(vtAvrg, 0);
  display.setCursor(4, 4);
  display.setTextSize(3);
  display.print(vFiltAvrg, 0);
  display.display();

  Serial.print(">pos:");
  Serial.print(pos);
  Serial.print(",v:");
  Serial.print(v);
  Serial.print(",vt:");
  Serial.print(vt);
  Serial.print(",avrgvt:");
  Serial.print(vtAvrg);
  Serial.print(",vFilt:");
  Serial.print(vFilt);
  Serial.print(",vFiltAvrg:");
  Serial.print(vFiltAvrg);
  Serial.print(",pwr:");
  Serial.print(pwr);
  Serial.println();
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