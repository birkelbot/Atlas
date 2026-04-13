#!/usr/bin/env python

# This program was built on Python 3.8.1.

################################################################################
# TODO: CODE NEEDS REVIEW
# 
# SUMMARY OF RECENT CHANGES (CRSF REFACTOR):
#
# - Transitioned serial output to use the CRSF protocol (24-byte packet, 420k baud).
#   WHY: Required to communicate directly with the RadioMaster Ranger Micro TX module 
#        over the JR bay connector using its native Crossfire format, replacing the
#        old custom serial Xbee architecture.
#
# - Serial port changed to '/dev/ttyAMA1' (default UART3 on Pi 4). 
#   WHY: The Pi 4 maps UART3 to Pins 7 & 29, which corresponds directly to the 
#        Ranger's TX/RX requirements.
#
# - Arcade drive logic (CH1, CH2) rescaled to output natively between 172 and 1811 (neutral 992).
#   WHY: The new ER5C-i receiver utilizes standard 11-bit CRSF channel data (172-1811) 
#        instead of the old 8-bit (0-254) integer logic used by the custom Arduino build.
#
# - Weapon Drum output (CH3) added and mapped to Right Bumper to toggle between 172 and 1811.
#   WHY: Atlas is equipped with a spinning drum that was not present on the old lifter. 
#        Mapping it to a bumper allows seamlessly toggling max power without moving 
#        thumbs off the primary drive sticks.
#
# - Arm Servo output (CH4) changed to slew target position up/down based on Analog Trigger pull.
#   WHY: Because CH4 drives a positional A20CLS servo instead of a direct-drive DC motor, 
#        sending pure analog trigger values would cause the servo to snap immediately to 
#        discrete angles. Integrating the trigger values into a persistent "slew" target 
#        creates a smooth, controllable arm sweep.
#
# - PyGame UI updated to explicitly represent these changes visually to the user.
################################################################################

# NEED TO ENABLE UART3 IN THE RASPBERRY PI CONFIG:
# * nano /boot/firmware/config.txt
# * Add dtoverlay=uart3
# * Reboot

# Connecting the Raspberry Pi to the RadioMaster Ranger Micro:
# * Pi Pin 7 (GPIO 4 / UART3 TX) -> Ranger JR Pin 1 (CRSF RX)
# * Pi Pin 6 (GND) -> Ranger JR Pin 4 (GND)
# * Pi Pin 29 (GPIO 5 / UART3 RX) -> Ranger JR Pin 5 (CRSF TX)

from enum import Enum
from numpy import interp
import pygame
import time
import os
import serial
import array
import sys
import math

# To check what serial ports are available in Linux, use the bash command: dmesg | grep tty
# To check what serial ports are available in Windows, go to Device Manager > Ports (COM & LPT)
# For Raspberry Pi 4 using UART3, the port is typically /dev/ttyAMA1
comPort = '/dev/ttyAMA1'
ser = serial.Serial(comPort, 420000, timeout=1)

### CONTROL SCHEME ###
# Drive:
#   Arcade Drive, i.e.
#     Left joystick Y-axis -- forward/reverse
#     Right joystick X-axis -- turn/arc
#
# Arm:
#   Right trigger -- arm pos target up (+ velocity)
#   Left trigger -- arm pos target down (- velocity)
# 
# Weapon:
#   Right Bumper -- Toggle Weapon on/off
#######################

# Set the channel numbers for various controls
AXIS_ID_DRIVE_VELOCITY = 1  # Y-axis translation comes from the left joystick Y axis
AXIS_ID_DRIVE_ROTATION = 2  # Rotation comes from the right joystick X axis
AXIS_ID_ARM = 0       # Analog triggers for arm control
ARM_USE_DUAL_ANALOG_INPUT = True  # Set to True/False for using single or dual analog triggers
AXIS_ID_ARM_UP =  4   # For computers that use separate channels for the analog triggers
AXIS_ID_ARM_DOWN =  5 # For computers that use separate channels for the analog triggers
BUTTON_ID_STOP_PROGRAM = 1
BUTTON_ID_WEAPON_TOGGLE = 7  # Right bumper to toggle weapon drum


############################################################
# @brief Class to help with printing text on a pygame screen.
############################################################
class TextPrint:
    def __init__(self, screen):
        self.screen = screen
        self.font = pygame.font.Font(None, 25)
        self.line_height = 20
        self.BLACK = (   0,   0,   0)
        self.WHITE = ( 255, 255, 255)
        self.reset()

    def disp(self, textString):
        textBitmap = self.font.render(textString, True, self.BLACK)
        self.screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.screen.fill(self.WHITE)
        self.x = 10
        self.y = 10
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10


############################################################
# @brief Class to print information on the driver station.
############################################################
class DriverStationScreen:
    def __init__(self):
        print("DriverStationScreen::init")
        screen = pygame.display.set_mode([400, 300])
        self.textPrint = TextPrint(screen)

    ############################################################
    ## @brief Display joystick inputs and subsystem commands
    ## @param yRaw - the raw joystick input for the Y-translation of the robot
    ## @param rRaw - the raw joystick input for the rotation of the robot
    ## @param armTrigs - the raw joystick trigger input for the arm shifting
    ## @param ch1 - Left drive command
    ## @param ch2 - Right drive command
    ## @param ch3 - Weapon drum command
    ## @param armTargetPos - the computed arm command
    ## @param packetsSent - the total number of packets sent so far to the robot
    ############################################################
    def updateDisplay(self, yRaw, rRaw, armTrigs, ch1, ch2, ch3, armTargetPos, packetsSent):
        self.textPrint.reset()

        self.textPrint.disp("ATLAS DRIVER STATION")
        self.textPrint.disp("SENDING CRSF PACKETS TO ROBOT")
        self.textPrint.disp("")  # Intentional blank line

        self.textPrint.disp("Raw Joystick Inputs (-1.0 <-> 1.0)")
        self.textPrint.indent()
        self.textPrint.disp("Y-translation raw: {:.3f}".format(float(yRaw)))
        self.textPrint.disp("Rotation raw: {:.3f}".format(float(rRaw)))
        self.textPrint.disp("Arm Trigger Shift: {:.3f}".format(float(armTrigs)))
        self.textPrint.unindent()
        self.textPrint.disp("")  # Intentional blank line

        self.textPrint.disp("CRSF Commands (172-1811, 992 is neutral)")
        self.textPrint.indent()
        self.textPrint.disp("CH1 (Left Drive): {}".format(int(ch1)))
        self.textPrint.disp("CH2 (Right Drive): {}".format(int(ch2)))
        self.textPrint.disp("CH3 (Weapon Drum): {}".format(int(ch3)))
        self.textPrint.disp("CH4 (Arm Servo): {:.1f}".format(float(armTargetPos)))
        self.textPrint.unindent()
        self.textPrint.disp("")
        self.textPrint.disp("Packets Sent: {}".format(packetsSent))

        pygame.display.flip()


def main():

    global ser

    pygame.init()

    # Create a UI for the driver station "game".
    # NOTE: In order for pygame to process bluetooth controller inputs, it needs
    #       to have a game that is in focus (i.e. the game must be the currently
    #       active window).
    screen = DriverStationScreen()
    pygame.display.set_caption("Driver Station")

    # Initialize the gamepad
    pygame.joystick.init()
    joysticks = []
    for i in range(pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))
        joysticks[i].init()
        print("Detected joystick '", joysticks[i].get_name(), "'")
        print("Joystick numaxes: ", joysticks[i].get_numaxes())

    # Local variables
    prevChannels = [992] * 16
    prevTimeSent = 0
    done = False
    packetsSent = 0
    armDualAnalogUpTriggerInitialized = False
    armDualAnalogDownTriggerInitialized = False

    # Atlas specific tracking state
    armTargetPos = 992.0 # Neutral CRSF pos for arm
    weaponOn = False
    prevWeaponBtn = False

    try:
        while (done == False):

            pygame.event.pump()  # This line is needed to process the gamepad packets

            if joystickWatchdog(joysticks[0]):
                sendNeutralCommand()
                continue

            ##### WHEEL COMMANDS (CH1, CH2) #####

            # Get the raw values for drive translation/rotation using the gamepad.
            yRaw = -joysticks[0].get_axis(AXIS_ID_DRIVE_VELOCITY)
            rRaw = joysticks[0].get_axis(AXIS_ID_DRIVE_ROTATION)

            # Get the drive motor commands for Arcade Drive (which scales to 172-1811)
            driveMtrCmds = arcadeDrive(yRaw, rRaw)

            ##########################

            ###### WEAPON DRUM (CH3) ######
            weaponBtn = joysticks[0].get_button(BUTTON_ID_WEAPON_TOGGLE)
            if weaponBtn and not prevWeaponBtn:
                weaponOn = not weaponOn
            prevWeaponBtn = weaponBtn
            
            weaponCmd = 1811 if weaponOn else 172

            ##########################

            ###### ARM SERVO COMMAND (CH4) #######

            # Get the raw values for the arm using the gamepad triggers
            armTrigs = 0
            if (ARM_USE_DUAL_ANALOG_INPUT):
                if (armDualAnalogUpTriggerInitialized and \
                    armDualAnalogDownTriggerInitialized):
                    # Positive output means "move up"
                    armTrigs = getArmRawFromDualAnalog( \
                        joysticks[0].get_axis(AXIS_ID_ARM_UP), \
                        joysticks[0].get_axis(AXIS_ID_ARM_DOWN))
                else:
                    if (joysticks[0].get_axis(AXIS_ID_ARM_UP) != 0):
                        armDualAnalogUpTriggerInitialized = True
                    if (joysticks[0].get_axis(AXIS_ID_ARM_DOWN) != 0):
                        armDualAnalogDownTriggerInitialized = True
            else:
                armTrigs = -joysticks[0].get_axis(AXIS_ID_ARM) # Invert if single axis

            # Compute dynamic target position (shift position based on trigger pull)
            max_speed = 10.0  # Max CRSF units per loop natively controls slew speed
            deadband = 0.05
            if abs(armTrigs) > deadband:
                armTargetPos += armTrigs * max_speed
                armTargetPos = max(172.0, min(1811.0, armTargetPos))

            ##########################

            if joysticks[0].get_button(BUTTON_ID_STOP_PROGRAM):
                cleanup()
                done = True
                
            # Populate our 16 channels for CRSF
            channels = [992] * 16
            channels[0] = int(driveMtrCmds['left'])
            channels[1] = int(driveMtrCmds['right'])
            channels[2] = weaponCmd
            channels[3] = int(armTargetPos)
            
            # Send at least every 20ms (50Hz packet rate minimum) or if channels changed
            if channels != prevChannels or time.time()*1000 > prevTimeSent + 20:

                frame = build_crsf_frame(channels)
                ser.write(frame)

                prevChannels = channels.copy()
                prevTimeSent = time.time()*1000

                packetsSent = packetsSent + 1
                screen.updateDisplay(yRaw, rRaw, armTrigs, channels[0], \
                                     channels[1], channels[2], armTargetPos, packetsSent)

                time.sleep(0.01)

    except KeyboardInterrupt:
        cleanup()


################################################################################
## @brief  Function to compute the drive motor PWM values for Arcade Drive
## @param  yIn - raw joystick input from -1.0 to 1.0 for the Y-axis translation
## @param  rIn - raw joystick input from -1.0 to 1.0 for the rotation
## @return an array containing left and right motor commands
################################################################################
def arcadeDrive(yIn, rIn):
    
    # Set output command range constants for CRSF
    zeroCommand = int(992)  # the neutral 1500us equivalent
    cmdRange = int(819)     # max variance from the zero command (992 -> 172 or 1811)
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential functions for each input (y/r)
    yExpConst = 1.5   # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    yEndpoint = 819   # maximum/minumum (+/-) for the Y-axis translation

    rExpConst = 2.75  # exponential growth coefficient of the rotation -- should be between 1.0-4.0
    rEndpoint = 500   # maximum/minimum (+/-) for the rotation

    endExpConst = 1.44 # don't change this unless you've really looked over the math

    # Set a deadband for the raw joystick input
    yDeadband = 0.08
    rDeadband = 0.05

    # Set a turning correction to help the robot drive straight (should be greater than the deadband)
    fwdTurningCorrection = 0
    revTurningCorrection = 0

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    leftMtrBaseCmd = int(40)
    rightMtrBaseCmd = int(40)

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if yIn < 0:
        yNeg = -1
    else:
        yNeg = 1

    if rIn < 0:
        rNeg = -1
    else:
        rNeg = 1

    # Apply a deadband
    if abs(yIn) < yDeadband:
        yIn = 0
    if abs(rIn) < rDeadband:
        rIn = 0

    # Apply a turning correction to help the robot drive straight
    if yIn > 0:
        rIn += fwdTurningCorrection
    if yIn < 0:
        rIn += revTurningCorrection

    # Compute the drive commands using the exponential function (zero-based)
    yCmd = \
      int( \
        ( \
          math.pow( \
            math.e, \
            math.pow(math.fabs(yIn), yExpConst) / endExpConst \
          ) \
          - 1 \
        ) \
        * yEndpoint \
        * yNeg \
      )
    rCmd = \
      int( \
        ( \
          math.pow( \
            math.e, \
            math.pow(math.fabs(rIn), rExpConst) / endExpConst \
          ) \
          - 1 \
        ) \
        * rEndpoint \
        * rNeg \
      )

    # Convert the drive commands into motor comands (zero-based)
    leftMtrCmd = yCmd + rCmd   # zero-based
    rightMtrCmd = yCmd - rCmd  # zero-based

    # Add an offset for the minimum command to overcome the gearboxes
    if leftMtrCmd > 0:
        leftMtrCmd = leftMtrCmd + leftMtrBaseCmd
    elif leftMtrCmd < 0:
        leftMtrCmd = leftMtrCmd - leftMtrBaseCmd
    if rightMtrCmd > 0:
        rightMtrCmd = rightMtrCmd + rightMtrBaseCmd
    elif rightMtrCmd < 0:
        rightMtrCmd = rightMtrCmd - rightMtrBaseCmd

    # If the commands are greater than the maximum or less than the minimum, scale them back
    maxMtrCmd = max(leftMtrCmd, rightMtrCmd)
    minMtrCmd = min(leftMtrCmd, rightMtrCmd)
    scaleFactor = 1.0
    if maxMtrCmd > maxCommand or minMtrCmd < minCommand:
        if maxMtrCmd > abs(minMtrCmd):
            scaleFactor = abs(float(maxCommand) / float(maxMtrCmd))
        else:
            scaleFactor = abs(float(minCommand) / float(minMtrCmd))

    leftdriveMtrCmdScaled = leftMtrCmd * scaleFactor
    rightdriveMtrCmdScaled = rightMtrCmd * scaleFactor

    # Shift the commands to be based on the zeroCommand (above)
    leftMtrCmdFinal = int(leftdriveMtrCmdScaled + zeroCommand)
    rightMtrCmdFinal = int(rightdriveMtrCmdScaled + zeroCommand)

    # Clamp safely inside CRSF range as double check
    leftMtrCmdFinal = max(172, min(1811, leftMtrCmdFinal))
    rightMtrCmdFinal = max(172, min(1811, rightMtrCmdFinal))

    return {'left':leftMtrCmdFinal, 'right':rightMtrCmdFinal}


############################################################
## @brief  Gets the raw arm input using separate analog triggers
##         for arm up and arm down.
## @param  aUp - analog trigger input for "arm up", where -1 is fully open and 1 is fully pressed
## @param  aDown - analog trigger input for "arm down", where -1 is fully open and 1 is fully pressed
## @return the overall raw arm command (-1 to 1)
############################################################
def getArmRawFromDualAnalog(aUp, aDown):
    aOut = 0
    if (aUp > -1):
        aOut = interp(aUp, [-1, 1], [0, 1])
    if (aDown > -1):
        aOut = -interp(aDown, [-1, 1], [0, 1])
    return aOut


############################################################
## @brief Run a watchdog check on the joystick
## @param joystick - the pygame joystick object
## @return true if the watchdog thinks the joystick died
############################################################
lastChangeDetected = time.time()*1000
prevAxes = []
prevBtns = []

def joystickWatchdog(joystick):
    global lastChangeDetected
    global prevAxes
    global prevBtns

    if not prevAxes:
        for i in range(0, joystick.get_numaxes()):
            prevAxes.append(joystick.get_axis(i))
    else:
        for i in range(0, joystick.get_numaxes()):
            if prevAxes[i] != joystick.get_axis(i):
                lastChangeDetected = time.time()*1000
            prevAxes[i] = joystick.get_axis(i)

    if not prevBtns:
        for i in range(0, joystick.get_numbuttons()):
            prevBtns.append(joystick.get_button(i))
    else:
        for i in range(0, joystick.get_numbuttons()):
            if prevBtns[i] != joystick.get_button(i):
                lastChangeDetected = time.time()*1000
            prevBtns[i] = joystick.get_button(i)

    # If no change happens in 7000ms, consider the joystick dead
    if time.time()*1000 > lastChangeDetected + 7000:
        return True
    else:
        return False


############################################################
## @brief Zero all the commands to the robot
############################################################
def sendNeutralCommand():
    global ser
    channels = [992] * 16
    channels[2] = 172 # keep weapon off!
    frame = build_crsf_frame(channels)
    ser.write(frame)

############################################################
## @brief Zero all the commands to the robot and exit
############################################################
def cleanup():

    global ser

    print("Cleaning up and exiting")
    sendNeutralCommand()
    ser.close()
    pygame.quit()
    exit()

############################################################
## CRSF Protocol specific functions 
############################################################
def crc8_dvb_s2(data):
    """Calculates CRC-8 using polynomial 0xD5."""
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0xD5
            else:
                crc <<= 1
            crc &= 0xFF
    return crc

def pack_rc_channels(channels):
    """
    Packs 16 channels (each 11 bits) into a 22-byte array.
    'channels' should be a list of 16 integers.
    """
    payload = bytearray(22)
    bits = 0
    bit_count = 0
    byte_index = 0
    
    for ch in channels:
        # Clamp channel to 11 bits (0-2047)
        bits |= (ch & 0x7FF) << bit_count
        bit_count += 11
        
        while bit_count >= 8:
            payload[byte_index] = bits & 0xFF
            bits >>= 8
            bit_count -= 8
            byte_index += 1
            
    return bytes(payload)

def build_crsf_frame(channels):
    """Builds the full CRSF frame for RC_CHANNELS_PACKED transmission."""
    # Frame Type: 0x16 for RC_CHANNELS_PACKED
    frame_type = 0x16
    payload = pack_rc_channels(channels)
    
    # Calculate CRC over Type + Payload
    crc_data = bytes([frame_type]) + payload
    crc = crc8_dvb_s2(crc_data)
    
    # Construct complete frame: [Address] [Length] [Type] [Payload] [CRC]
    # Length = Payload Size (22) + 2 (Type + CRC) = 24
    # Address 0xC8 is standard Flight Controller address to Transmitter
    frame = bytes([0xC8, 24, frame_type]) + payload + bytes([crc])
    return frame

if __name__ == '__main__':
    sys.exit(int(main() or 0))
