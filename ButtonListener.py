#!/usr/bin/env python

import os
from gpiozero import Button

button = Button(3)

while True:
    button.wait_for_press()
    print("button pressed!")
    os.system("python3 /home/pi/Lockout/DriverStation/DriverStation.py")
    button.wait_for_release()
