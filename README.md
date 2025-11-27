Overview -- Its baically a drawing bot Upload a picture, extract line coordinates via OpenCV, and our Arduino-powered arm will plot it using inverse kinematics and servo control.

This was something like a hobby fun thing I  built in 1st yr summer. 

Mungai is a kannada word for hand or wrist 

Involves open-cv image processing and and use of microcontrollers, motor drivers and other actuators and some inverse kinematics overall very fun 

So i got inspired from cnc machine that if i upload a image it should draw me the rough sketch of that image, which is cool though to think and i bought some components thanks to robu.in i built in 3 to 4 days 

Simple project i have included  the lines.py file gives an array of all the major lines in the image and the main file is uploaded on arduino which has a bit of inverse kinematics to use 3 servo motors which are made like an arm and the lines are drawn. 

🚀 Features

Convert any input image into drawable paths

Detect edges and contours using OpenCV

Convert pixel coordinates into robot-friendly servo angle commands

Communicate with Arduino over serial

Control 2-3 servos to replicate the drawing motion

Modular code for easy modification and experimentation


🧩 System Architecture

1. Image Processing (Python)

Read and resize input image

Apply Canny edge detection / thresholding

Extract contours

Convert contours → (x, y) coordinate list

Send coordinates to Arduino over serial

2. Embedded Control (Arduino)

Receive coordinate pairs

Convert coordinates → servo rotation angles

Move drawing arm to trace the image

Control pen up/down for actual drawing


🛠 Technologies Used

Python

OpenCV (image processing)

PySerial (Arduino communication)

Arduino C++ (servo control)

Servo motors, Arduino Uno/Nano



