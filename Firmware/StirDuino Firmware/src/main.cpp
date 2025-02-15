/*****************************************************************************************
*  ____  ____  __  ____  ____  _  _  __  __ _   __  
* / ___)(_  _)(  )(  _ \(    \/ )( \(  )(  ( \ /  \ 
* \___ \  )(   )(  )   / ) D () \/ ( )( /    /(  O )
* (____/ (__) (__)(__\_)(____/\____/(__)\_)__) \__/ 
*
* Firmware for Arduino based magnetic stirrer featuring PI feedback loop and GUI (OLED). 
* Compatible with Arduino Nano and Nano Every.
*
* Code written by Ludwig Gabler
*****************************************************************************************/

#include <Arduino.h>
#include <util/atomic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// Pins
#define OPTICAL_ENC 8     // Optical Encoder
#define EN 3      // Pin to enable motor driver (pwm) !!! NOT 2, because this pin has no pwm !!!
#define PH 2      // Pin to set direction
#define POT A0    // Wiper terminal of 10k potentiometer
// #define ENC_SW 7  // Switch of rotary encoder switch
// #define ENC_DT 6  // DT Pin of rotary encoder switch
// #define ENC_CLK 5 // CLK Pin of rotary encoder switch

#define OPTICAL_ENC_PULSES 120       // number of pulses of optical encoder
#define MAX_RPM 1400                // maximum speed. If the speed is too high, the encoder ticks can no longer be registered.
#define MIN_RPM 0                   // mminimum speed. Speeds that are too low can be unstable.
#define CONTROLLER_REFRESH_RATE 200 // Frequency for updating the PI control values

// Serial interface
#define BAUD_RATE 115200
#define SERIAL_SEND_RATE 10   // Hz

// Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define DISPLAY_REFRESH_RATE 10 // Hz

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


/******************************************************************************* 
 * Sets motor speed and direction through the use of analogWrite and digitalWrite.
 *
 * Inputs:
 * - direction (1: CW, -1: CCW)
 * - PWM duty cycle (0 - 255)
 * - PWM pin
 * - direction pin
*******************************************************************************/
void setMotor(int dir, int pwmVal, int pwm, int phase);

/*******************************************************************************
 *  Increases position counter by one. Called by interrupt attached to optical
 * encoder.
********************************************************************************/
void readEncoder();

// globals
uint16_t lastContrUpdate = 0;
uint16_t lastDispUpdate = 0;
uint16_t lastSerUpdate = 0;

int pos = 0;  // tick counter of optical encoder
float v = 0;  // unfiltered rotational speed (rpm)
int pwr = 0;  // PWM duty cycle (0 - 255)

long prevT = 0;   // last time PI loop ran (µs)
int posPrev = 0;  // last tick count of optical encoder

float vt = 0;     // target rotational speed set by potentiometer (rpm)
float vFilt = 0;  // filtered rotational speed (rpm)
float vPrev = 0;  // last rotational speed (rpm)

float eintegral = 0;  // integral term of PI controller

volatile int pos_i = 0; // volatile for variables used in an interrupt

// variables for moving average calculation for more stable values
// displayed on OLED display
const int avrgCount = 64;         // sampling width of mooving average
float vtAvrgBuffer[avrgCount];    // buffer for target speed average
int vtNextAvrg = 0;               // sample count for target speed average
float vtAvrg = 0;                 // target speed average
float vFiltAvrgBuffer[avrgCount]; // buffer for filtered speed average
int vFiltNextAvrg = 0;            // sample count for filtered speed average
float vFiltAvrg = 0;              // filtered speed average


void setup() {

  // Change devider for clock to set PWM frequency to 64.5 kHz
  // (also affects tone() if used)
  TCB1_CTRLA = 0b00000001;

  Serial.begin(BAUD_RATE);
  Wire.begin();

  pinMode(OPTICAL_ENC, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(POT, INPUT);

  // Interrupt for optical encoder
  attachInterrupt(digitalPinToInterrupt(OPTICAL_ENC), readEncoder, RISING);

  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.setRotation(2);
  display.display();
  display.clearDisplay();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);

  Serial.println("StirDuino Firmware v1.0");
}

void loop() {
  
  uint16_t current = millis();

  // update control loop
  if ((current - lastContrUpdate) >= (1.0e3 / CONTROLLER_REFRESH_RATE)) {

    // read in atomic block so value cant change while being read
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = pos_i;
    }

    long currT = micros();                              // current time in µs
    float deltaT = ((float) (currT - prevT)) / 1.0e6;   // time since last loop in s
    float velocity = (pos - posPrev) / deltaT;          // current rotational speed (pulses per second)
    posPrev = pos;
    prevT = currT;

    v = velocity / OPTICAL_ENC_PULSES * 60.0;           // Convert counts/s to RPM

    // Low-pass filter (25 Hz cutoff) (should be recalculated with actual data)
    vFilt = 0.854*vFilt + 0.0728*v + 0.0728*vPrev;
    vPrev = v;

    // moving averages for more stable readings on display
    vFiltAvrgBuffer[vFiltNextAvrg++] = vFilt;
    if (vFiltNextAvrg >= avrgCount) {
      vFiltNextAvrg = 0;
    }
    vFiltAvrg = 0;
    for (int i = 0; i < avrgCount; i++) {
      vFiltAvrg += vFiltAvrgBuffer[i];
    }
    vFiltAvrg /= avrgCount;

    // calculate target speed from potentiometer reading (noisy)
    int pot = analogRead(POT);
    vt = map(pot, 0, 1023, MIN_RPM, MAX_RPM + 4);
    if (vt < 10) {  // avoid too low speeds
      vt = 0;
    }

    // calculate moving average for target speed
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
      dir = 1;
    }
    pwr = (int) fabs(u);
    if (pwr > 255) {
      pwr = 255;
    }
    if (vt < 10) {      // avoid too low speeds
      pwr = 0;
    }
    setMotor(dir, pwr, EN, PH);

    lastContrUpdate = current;
  }
  
  // update display
  if ((current - lastDispUpdate) >= (1.0e3/DISPLAY_REFRESH_RATE)) {
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
    lastDispUpdate = current;
  }

  // send values over serial interface
  // formatted for use with serial-plotter (by Mario Zechner)
  if ((current - lastSerUpdate) >= (1.0e3/SERIAL_SEND_RATE)) {
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
    lastSerUpdate = current;
  }
}

void setMotor(int dir, int pwmVal, int pwm, int phase) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(phase, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(phase, LOW);
  }
}

void readEncoder() {
  pos_i++;
}