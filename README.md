# Secure-Wireless-Locking-System
Embedded Systems Project from CS122A

This project simulates a wireless locking mechanism. The RFID tag is read and validated on one end of the system, and a signal is sent to the receiver to lock or unlock the target device. Project is written in Embedded-C. I utilized 2 ATmega1284 microcontrollers, 2 hc-05 Bluetooth modules, 1 RFID card reader, and 2 LCD screens. 

Link to video demonstration: https://www.youtube.com/watch?v=le9lqEhSdYQ&t=1s

main.c is the code for the first microcontroller to read RFID input, send data over Bluetooth, and display to Adafruit LCD.
main_2.c is the code for the second microcontroller for receiving data over Bluetooth and displaying status to Nokia 5110 LCD.
