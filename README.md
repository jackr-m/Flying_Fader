# Flying Faders
## Use Rust Embassy framework to integrate 5 "flying faders" (motorized linear slide potentiometers) as a USB MIDI CC controller

- [Flying Faders](#flying-faders)
   * [Use Rust Embassy framework to integrate 5 "flying faders" (motorized linear slide potentiometers) as a USB MIDI CC controller](#use-rust-embassy-framework-to-integrate-5-flying-faders-motorized-linear-slide-potentiometers-as-a-usb-midi-cc-controller)
   * [Project Overview](#project-overview)
      + [Code](#code)
      + [Mechanical](#mechanical)

![Final product](https://github.com/jackr-m/Flying_Fader/blob/master/photos/Final.jpg?raw=true)

## Project Overview
Use 5 motorized linear slide potentiometers (ALPS SM10002NKA1X-HW1, sold as Adafruit P/N 5466) with a STM32H7 microprocessor programmed using the Rust Embassy framework & HAL.
There is a custom PCB for this project, with the KiCad source in the /CAD directory.
![PCB](https://github.com/jackr-m/Flying_Fader/blob/master/photos/PCB.png?raw=true)

### Code
Using the Rust Embassy framework and embassy_stm32 HAL, this has been coded using an asynchronous architecture and should be very portable to nearly any STM32 variant (especially if the screen is not needed).
Code documentation located at: https://jackr-m.github.io/Flying_Fader/flying_fader/

### Mechanical
The casement is 3D printed, and STEP files are provided in the /CAD directory.
In addition, the original Onshape document is available for use or modification at [Onshape](https://cad.onshape.com/documents/5790ae0e6f82956a005dc546/w/d37346e9ef2c1592c912aa6b/e/dacfcc9dc0c38682bf8baa7f?renderMode=0&uiState=666d3492034c5f73c5a59cf9).
![CAD](https://github.com/jackr-m/Flying_Fader/blob/master/photos/CAD.png?raw=true)


Faders:
![Fader](https://github.com/jackr-m/Flying_Fader/blob/master/photos/5466-01.jpg?raw=true)
https://www.adafruit.com/product/5466

Screen:
![Screem](https://github.com/jackr-m/Flying_Fader/blob/master/photos/5805-00.jpg?raw=true)
https://www.adafruit.com/product/5805
