# TP-DIGITAL2

Este programa para el PIC16F887 consiste en un sistema que enfria o calienta agua hasta alcanzar una temperatura inicial de 25 ºC y que se puede configurar en tiempo real a 
traves de un teclado, todo esto mientras se muestra la temperatura del agua por dos displays de 7 segmentos. El programa esta pensado para temperaturas de entre 0 y 99 ºC.

Se hace uso del Timer0, del Timer1, interrupciones por ambos, el ADC y el modulo EUSART para transmision en serie. Ademas se realiza multiplexado de teclado y display.
