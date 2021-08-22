![Picture1](https://user-images.githubusercontent.com/44223263/130352462-f32a7a5e-0e2f-432c-8049-398d41250e65.jpg)

# ESP32cam-motion-detection
This is a motion detection system built on the ESP32-CAM board and a PIR sensor module that also sends an email with the image as an attachment. 

The board spends most of the time in sleep mode and wakes up to take an image once motion is detected. Once motion is detected, an interrupt routine is triggered. It takes the picture of the intruder or whatever, saves it on the file memory system of the board called SPIFFs, then using STMP protocols, it sends the image to a designated mail.

# Materials used in this Project

>
> * ESP32-camera
> * PIR sensor (generic)
> * Power Jack
> * 9v battery and connector
> * 1kohm and 10kohm resistors, NPN transistor (2N3904)
> * Header Pins
> * Switch
> * Potentiometer and KA317 voltage regulator
> * Wires, soldering and copper through hole PCB

# How to use
+ Make the necessary connections as shown in the circuit diagram
+ Upload .ino file to ESP32
+ Edit code with your preferred mail, and your ssid credentials
