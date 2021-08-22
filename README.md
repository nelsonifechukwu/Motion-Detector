![ESP32-CAM-PIR-Motion-Sensor-Wiring](https://user-images.githubusercontent.com/44223263/130352393-a17de108-9452-4c8c-b50e-b2586f79fe3e.jpeg)
# ESP32cam-motion-detection
This is a motion detection system built on the ESP32-CAM board and a PIR sensor module that also sends an email with the image as an attachment. 

The board spends most of the time in sleep mode and wakes up to take an image once motion is detected. Once motion is detected, an interrupt routine is triggered. It takes the picture of the intruder or whatever, saves it on the file memory system of the board called SPIFFs, then using STMP protocols, it sends the image to a designated mail.

+ Make the necessary connections as shown in the circuit diagram
+ Upload .ino file to ESP32
+ Edit code with your preferred mail, and your ssid credentials
