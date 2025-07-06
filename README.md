# Water Temperature Control System for PIC16F887

This application, developed for the PIC16F887 microcontroller, implements a system capable of heating or cooling water to reach a target temperature, initially set to 25 °C. The desired temperature can be configured in real time via a keypad, while the current water temperature is continuously displayed on two 7-segment displays. The system is designed to operate within a temperature range of 0 to 99 °C.

The implementation utilizes Timer0 and Timer1, including their respective interrupt features, the Analog-to-Digital Converter (ADC), and the Enhanced Universal Synchronous Asynchronous Receiver Transmitter (EUSART) module for serial communication. Furthermore, the design incorporates multiplexing for both the keypad and the display.
