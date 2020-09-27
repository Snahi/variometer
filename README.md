# Barovario
## Introduction
This project is an Arduino variometer which obtains the current vertical speed from a barometer. Speed and height above the ground level (baro altitude) are displayed on an OLED display. Besides display, this variometer is also equipped with a speaker which emits different sounds depending on the vertical speed. The delay is designed to be 800 ms, because this way we can obtain a precise vertical speed measure. When the delay was smaller (~300ms) the variometer tended to show wrong values. With the acquired precision it is possible to detect even a 0.1 m/s vertical speed.

## Used components
* Arduino UNO
* Grove - Barometer Sensor (BMP280)
* Grove - OLED Display 0.96"
* Speaker

## Pins

### Barometer
* **GND** GND
* **VCC** 3.3V
* **SDA** Pin A4
* **SCL** Pin A5
### Display
* **GND** GND
* **VCC** 3.3V or 5V
* **SDA** Pin A4
* **SCL** Pin A5
### Speaker
* **+** Pin 11
* **-** GND

## Code
The code consists of **barovario.ino** and third party libraries (with small modifications). The **barovario.ino** file is where the main logic is implemented. The libraries are just interfaces to the barometer and the display. The libraries were taken from Groove, but small modifications were made to them, so for the best results the libraries from the *libraries* directory should be used.
