# Atlas

## Overview
This repo contains the control code for Atlas, a 1-lb Plastic Antweight combat robot. Its primary weapon system is a lifter arm that features a spinning drumlet at the end. 

## Driver Station & Communications
The driver station is powered by a Raspberry Pi 4 running the `DriverStation.py` control software with a connected Xbox One Series S|X controller. The Python script generates and sends 24-byte CRSF (Crossfire) protocol packets to a **RadioMaster Ranger Micro** transmitter module.

### Wiring: Raspberry Pi 4 to Ranger Micro
The Ranger Micro module receives CRSF serial data over its JR bay connector. The Pi utilizes its hardware UART3 port for this communication running at `420000` baud.

* **Pi Pin 7 (GPIO 4 / UART3 TX)** -> **Ranger JR Pin 1 (CRSF RX)**
* **Pi Pin 6 (GND)** -> **Ranger JR Pin 4 (GND)**
* **Pi Pin 29 (GPIO 5 / UART3 RX)** -> **Ranger JR Pin 5 (CRSF TX)**

*NOTE: UART3 must be enabled in `/boot/firmware/config.txt` by adding `dtoverlay=uart3`. This typically exposes the serial interface on `/dev/ttyAMA1`.*

## Robot Electronics
Inside the robot, a **RadioMaster ER5C-i** receiver pulls in the CRSF communication.

The receiver channels are mapped as follows:
* **CH1 (Left Drive):** Driven by the Left analog stick (Arcade Drive mapping). Valid range `172-1811` where `992` is no power.
* **CH2 (Right Drive):** Driven by the Right analog stick (Arcade Drive mapping). Valid range `172-1811` where `992` is no power.
* **CH3 (Weapon Drumlet):** One-directional spinning drum. Toggled on to max power (`1811`) and off (`172`) using the Right Bumper on the gamepad.
* **CH4 (Arm Servo):** Powered by an **AGFRC A20CLS** servo. Because it is a positional servo rather than a direct-drive DC motor, the code integrates the analog trigger values to slew the target position smoothly over time, avoiding harsh snapping motions.

## Usage
To run the driver station UI and begin transmitting CRSF packets:
```bash
python3 DriverStation.py
```
*Note: A Xbox One Series S|X controller must be connected, and the generated PyGame UI window must remain in focus to properly process driver inputs.*