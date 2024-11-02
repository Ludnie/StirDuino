# StirDuino
Arduino based, small form factor, low cost magnetic stir plate with PID-controlled stir speed.

## Features
- **Arduino based:** The stir plate is controlled by a Arduino nano µ-controller. A low cost and widely availible development board with extensive documentation. The firmware code is written in C++ and can be easily modified via the Arduino IDE.
- **OLED-Display:** Stir speed and set speed are highly visible displayed on a 0.9" OLED-Display.
- **Small Formfactor:** Footprint of only 70 mm x 70 mm.
- **3D-Printed Enclosure:** Enclosure is cost effectively printed. Material can be chosen depending on the use case.
- **Stir Speed:** min.: 50 rpm, max.: 1400 rpm
- **Neodynium Magnets:** Uses powerful rare earth magnets for maximum stir bar retention.
- **Remote Controlled:** Stir parameters can be remotely set via the serial interface of the µ-controller.
- **Serial Communication:** Stir parameters can be requested and logged via the serial port of the µ-controller.

## BOM
| Part               | Quantity  |
|--------------------|-----------|
| Arduino nano every |     1     |

## Upgrades and Fixes for next Version
- [ ] Implement **operating hours counter**. Operating time stored in eeprom.
- [ ] Add funktionality to **send commands** to stir plate via the serial interface. These commands may include:
    - [ ] Change mode from user input to serial input.
    - [ ] Set stir speed.
    - [ ] Set stir direction.
    - [ ] Request current stir speed.
    - [ ] Request current stir direction.
    - [ ] Request current moter power (PWM duty cycle)
    - [ ] Request current configuration Parameters (Firmware Version, operating hours, PID-Params, etc.)
- [ ] Replace potentiometer with **rotary encoder/switch**. PCB is allready designed with neccessary traces and footprint for a connector. This will enable easier input for set point and implementation of a basic menu.
- [ ] Improve fastening method for **OLED-display**.
- [ ] Put **knob** into a recessed position.
- [ ] **Liquid proof** the device (especially display).
- [ ] Implement some mechanism for **detecting a "slipping" stir bar**.