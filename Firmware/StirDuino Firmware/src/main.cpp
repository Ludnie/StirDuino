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

// Libraries
#include <Arduino.h>
#include <util/atomic.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <PID_v1.h>

// Debugging options (taken from https://forum.arduino.cc/t/managing-serial-print-as-a-debug-tool/1024824/2)
#define DEBUG 0 // SET TO 0 OUT TO REMOVE TRACES

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__)
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_write(...)
#define D_println(...)
#endif

// Pins
#define OPTICAL_ENC 8     // Optical Encoder
#define EN 3      // Pin to enable motor driver (pwm) !!! NOT 2, because this pin has no pwm !!!
#define PH 2      // Pin to set direction
#define POT A0    // Wiper terminal of 10k potentiometer
// #define ENC_SW 7  // Switch of rotary encoder switch
// #define ENC_DT 6  // DT Pin of rotary encoder switch
// #define ENC_CLK 5 // CLK Pin of rotary encoder switch

#define OPTICAL_ENC_PULSES 120      // number of pulses of optical encoder
#define MAX_RPM 1500                // maximum speed. If the speed is too high, the encoder ticks can no longer be registered.
#define MIN_RPM 0                   // mminimum speed. Speeds that are too low can be unstable.
#define CONTROLLER_REFRESH_RATE 20  // Frequency for updating the PI control values
#define aggP 0.5                    // aggressive Proportional gain
#define aggI 0.2                    // aggressive Integral gain
#define aggD 0.2                    // aggressive Derivative gain
#define consP 0.3                   // conservative Proportional gain
#define consI 0.1                   // conservative Integral gain
#define consD 0.1                   // conservative Derivative gain

// Serial interface
#define BAUD_RATE 115200
#define SERIAL_SEND_RATE 1   // Hz

// Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C
#define DISPLAY_REFRESH_RATE 2 // Hz

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

/*******************************************************************************
 *  Right-aligns a float value for printing.
 *
 * Inputs:
 * - float value
 * - width of the string
 * - number of decimal places
 *
 * Returns:
 * - right-aligned string
 * ******************************************************************************/
String floatAlignRight(float f, int width, int precision);

// globals
uint16_t lastContrUpdate = 0;
uint16_t lastDispUpdate = 0;
uint16_t lastSerUpdate = 0;

int pos = 0;  // tick counter of optical encoder
float v = 0;  // unfiltered rotational speed (rpm)
int pwr = 0;  // PWM duty cycle (0 - 255)

long prevT = 0;   // last time PI loop ran (µs)
int posPrev = 0;  // last tick count of optical encoder

float alpha = 0.075;  // low-pass filter coefficient
float vt = 0;         // target rotational speed set by potentiometer (rpm)
float vFilt = 0;      // filtered rotational speed (rpm)
float vPrev = 0;      // last rotational speed (rpm)

// PID controller
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, consP, consI, consD, DIRECT);

volatile int pos_i = 0; // volatile for variables used in an interrupt

// variables for moving average calculation for more stable values
// displayed on OLED display
const int avrgCount = 32;         // sampling width of mooving average
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

  // Initialize serial interface and I2C bus
  Serial.begin(BAUD_RATE);
  Wire.begin();

  // Initialize pins
  pinMode(OPTICAL_ENC, INPUT);
  pinMode(EN, OUTPUT);
  pinMode(PH, OUTPUT);
  pinMode(POT, INPUT);

  // Interrupt for optical encoder
  attachInterrupt(digitalPinToInterrupt(OPTICAL_ENC), readEncoder, RISING);

  // Initialize display
  display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
  display.clearDisplay();
  display.setRotation(2);
  display.display();
  display.setTextSize(3);
  display.setTextColor(SSD1306_WHITE);

  // Initialize PID controller
  Setpoint = 0;
  Input = 0;
  myPID.SetMode(AUTOMATIC);

  Serial.println("StirDuino Firmware v1.0");

  // Display splash screen
  display.setCursor(0, 0);
  display.setTextSize(2);
  display.print("StirDuino");
  display.setTextSize(1);
  display.setCursor(0, 24);
  display.print("Firmware v1.0");
  display.display();
  delay(5000);
}

void loop() {
  
  // current time in ms
  uint16_t current = millis();

  // update control loop
  if ((current - lastContrUpdate) >= (1.0e3 / CONTROLLER_REFRESH_RATE)) {

    // read in atomic block so value cant change while being read
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      pos = pos_i;
    }

    // calculate current speed
    long currT = micros();                              // current time in µs
    float deltaT = ((float) (currT - prevT)) / 1.0e6;   // time since last loop in s
    float velocity = (pos - posPrev) / deltaT;          // current rotational speed (counts/s)
    posPrev = pos;                                      // update last tick count
    prevT = currT;                                      // update last time
    v = velocity / OPTICAL_ENC_PULSES * 60.0;           // Convert counts/s to RPM

    // Low-pass filter
    vFilt = alpha*v + (1-alpha)*vPrev;
    vPrev = vFilt;
    Input = vFilt;

    // calculate target speed from potentiometer reading (noisy)
    int pot = analogRead(POT);
    vt = map(pot, 0, 1023, MIN_RPM, MAX_RPM + 4);
    if (vt < 10) {  // avoid too low speeds
      vt = 0;
    }
    Setpoint = vt;

    // Use aggressive PID values for small errors and conservative values for large errors
    double gap = abs(Setpoint - Input);
    if (gap < 20) {
      myPID.SetTunings(consP, consI, consD);
    }
    else {
      myPID.SetTunings(aggP, aggI, aggD);
    }
    
    // Compute PID output
    myPID.Compute();

    // Set motor speed and direction
    int dir = 1;
    if (Output<0) {
      dir = -1;
      //Output = 0;
    }

    // limit power to 255
    pwr = (int) fabs(Output);
    if (pwr > 255) {
      pwr = 255;
    }

    // avoid too low speeds
    if (vt < 10) {
      pwr = 0;
    }

    // set motor speed and direction
    setMotor(dir, pwr, EN, PH);

    // update last update time
    lastContrUpdate = current;

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

    // Prints for debugging
    D_println(">pos:" + String(pos) 
            // + ", currT:" + String(currT) 
            // + ", deltaT:" + String(deltaT, 6) 
            // + ", velocity:" + String(velocity, 6) 
            + ", v:" + String(v) 
            + ", vFilt:" + String(vFilt) 
            // + ", vFiltAvrg:" + String(vFiltAvrg) 
            + ", vt:" + String(vt) 
            // + ", vtAvrg:" + String(vtAvrg)
            + ", Input:" + String(Input)
            + ", Output:" + String(Output)
            // + ", dir:" + String(dir)
            + ", pwr:" + String(pwr));
  }
  
  // update display
  if ((current - lastDispUpdate) >= (1.0e3/DISPLAY_REFRESH_RATE)) {
    display.clearDisplay();
    display.setCursor(80, 2);
    display.setTextSize(1);
    display.print("rpm  SET");
    display.setCursor(80, 12);
    display.setTextSize(2);
    display.print(floatAlignRight(vtAvrg, 4, 0));
    display.setCursor(4, 4);
    display.setTextSize(3);
    display.print(floatAlignRight(vFiltAvrg, 4, 0));
    display.display();
    lastDispUpdate = current;
  }

  // send values over serial interface
  // formatted for use with serial-plotter (by Mario Zechner)
  if ((current - lastSerUpdate) >= (1.0e3/SERIAL_SEND_RATE)) {
    Serial.println(">v:" + String(v) + ", vt:" + String(vt) + ", vFilt:" + String(vFilt) + ", pwr:" + String(pwr));
    lastSerUpdate = current;
  }
}

// Functions

// Sets motor speed and direction
// dir: direction (1: CW, -1: CCW)
// pwmVal: PWM duty cycle (0 - 255)
// pwm: PWM pin
// phase: direction pin
void setMotor(int dir, int pwmVal, int pwm, int phase) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(phase, HIGH);
  }
  else if (dir == -1) {
    digitalWrite(phase, LOW);
  }
}

// Interrupt function for optical encoder
// Increases position counter by one
void readEncoder() {
  pos_i++;
}

// Function for right-aligned float printing
String floatAlignRight(float f, int width, int precision) {
  String s = String(f, precision);
  int len = s.length();
  for (int i = 0; i < width - len; i++) {
    s = " " + s;
  }
  return s;
}