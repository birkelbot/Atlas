#!/usr/bin/env python

# This program was built on Python 3.8.1.

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
comPort = '/dev/ttyUSB0'
ser = serial.Serial(comPort, 57600, timeout=1)

### CONTROL SCHEME ###
# Drive:
#   Arcade Drive, i.e.
#     Left joystick Y-axis -- forward/reverse
#     Right joystick X-axis -- turn/arc
#
# Arm:
#   Right trigger -- arm up (analog control)
#   Left trigger -- arm down (analog control)
#######################

# Set the channel numbers for various controls
AXIS_ID_DRIVE_VELOCITY = 1  # Y-axis translation comes from the left joystick Y axis
AXIS_ID_DRIVE_ROTATION = 2  # Rotation comes from the right joystick X axis
AXIS_ID_ARM = 0       # Analog triggers for arm control
ARM_USE_DUAL_ANALOG_INPUT = True  # Set to True/False for using single or dual analog triggers
AXIS_ID_ARM_UP =  4   # For computers that use separate channels for the analog triggers
AXIS_ID_ARM_DOWN =  5 # For computers that use separate channels for the analog triggers
BUTTON_ID_STOP_PROGRAM = 1
BUTTON_ID_ARM_TURBO = 0  # "A" button
BUTTON_ID_ARM_DOWN_SLOW = 6  # Left bumper
BUTTON_ID_ARM_UP_SLOW = 7  # Right bumper


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
    ## @brief Display joystick inputs and motor commands
    ## @param yRaw - the raw joystick input for the Y-translation of the robot
    ## @param rRaw - the raw joystick input for the rotation of the robot
    ## @param armRaw - the raw joystick input for the arm
    ## @param lMtrCmd - the computed left motor command
    ## @param rMtrCmd - the computed right motor command
    ## @param armCmd - the computed arm command
    ## @param packetsSent - the total number of packets sent so far to the robot
    ############################################################
    def updateDisplay(self, yRaw, rRaw, armRaw, lMtrCmd, rMtrCmd, armCmd, packetsSent):
        self.textPrint.reset()

        self.textPrint.disp("KEEP THIS WINDOW ACTIVE TO CONTINUE")
        self.textPrint.disp("SENDING COMMANDS TO THE ROBOT")
        self.textPrint.disp("")  # Intentional blank line

        self.textPrint.disp("Raw Joystick Inputs (-1.0 <-> 1.0)")
        self.textPrint.indent()
        self.textPrint.disp("Y-translation raw: {}".format(yRaw))
        self.textPrint.disp("Rotation raw: {}".format(rRaw))
        self.textPrint.disp("Arm raw: {}".format(armRaw))
        self.textPrint.unindent()
        self.textPrint.disp("")  # Intentional blank line

        self.textPrint.disp("Motor Commands (0 <-> 254, 127 is neutral)")
        self.textPrint.indent()
        self.textPrint.disp("Left drive motor: {}".format(lMtrCmd))
        self.textPrint.disp("Right drive motor: {}".format(rMtrCmd))
        self.textPrint.disp("Arm motor: {}".format(armCmd))

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
    prevDriveMtrCmds = {'left':0, 'right':0}
    prevArmCmd = 127
    prevTimeSent = 0
    done = False
    packetsSent = 0
    armDualAnalogUpTriggerInitialized = False
    armDualAnalogDownTriggerInitialized = False

    try:
        while (done == False):

            pygame.event.pump()  # This line is needed to process the gamepad packets

            if joystickWatchdog(joysticks[0]):
                sendNeutralCommand()
                continue

            ##### WHEEL COMMANDS #####

            # Get the raw values for drive translation/rotation using the gamepad.
            yRaw = -joysticks[0].get_axis(AXIS_ID_DRIVE_VELOCITY)
            rRaw = joysticks[0].get_axis(AXIS_ID_DRIVE_ROTATION)

            # Get the drive motor commands for Arcade Drive
            driveMtrCmds = arcadeDrive(yRaw, rRaw)

            ##########################

            ###### ARM COMMAND #######

            # Get the raw values for the arm using the gamepad
            armRaw = 0
            if (ARM_USE_DUAL_ANALOG_INPUT):
                if (armDualAnalogUpTriggerInitialized and \
                    armDualAnalogDownTriggerInitialized):
                    armRaw = -getArmRawFromDualAnalog( \
                        joysticks[0].get_axis(AXIS_ID_ARM_UP), \
                        joysticks[0].get_axis(AXIS_ID_ARM_DOWN))
                else:
                    if (joysticks[0].get_axis(AXIS_ID_ARM_UP) != 0):
                        armDualAnalogUpTriggerInitialized = True
                    if (joysticks[0].get_axis(AXIS_ID_ARM_DOWN) != 0):
                        armDualAnalogDownTriggerInitialized = True
            else:
                armRaw = joysticks[0].get_axis(AXIS_ID_ARM)

            # NOTE: Choose linear or exponential drive by changing between
            #       `manualArmLinDrive()` and `manualArmExpDrive()`
            armCmd = 254 - manualArmExpDrive( \
                -armRaw, joysticks[0].get_button(BUTTON_ID_ARM_TURBO), \
                joysticks[0].get_button(BUTTON_ID_ARM_DOWN_SLOW), \
                joysticks[0].get_button(BUTTON_ID_ARM_UP_SLOW))

            ##########################

            if joysticks[0].get_button(BUTTON_ID_STOP_PROGRAM):
                cleanup()
                done = True
             # Only send if the commands changed or if 50ms have elapsed
            elif prevDriveMtrCmds['left'] != driveMtrCmds['left'] or \
                 prevDriveMtrCmds['right'] != driveMtrCmds['right'] or \
                 prevArmCmd != armCmd or \
                 time.time()*1000 > prevTimeSent + 50:

                ser.write((255).to_bytes(1, byteorder='big'))  # Start byte
                ser.write((driveMtrCmds['left']).to_bytes(1, byteorder='big'))
                ser.write((driveMtrCmds['right']).to_bytes(1, byteorder='big'))
                ser.write((254-armCmd).to_bytes(1, byteorder='big'))

                prevDriveMtrCmds = driveMtrCmds
                prevArmCmd = armCmd
                prevTimeSent = time.time()*1000

                packetsSent = packetsSent + 1
                screen.updateDisplay(yRaw, rRaw, armRaw, driveMtrCmds['left'], \
                                     driveMtrCmds['right'], armCmd, packetsSent)

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
    
    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential functions for each input (y/r)
    yExpConst = 1.5  # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    yEndpoint = 127  # maximum/minumum (+/-) for the Y-axis translation

    rExpConst = 2.75  # exponential growth coefficient of the rotation -- should be between 1.0-4.0
    rEndpoint = 90   # maximum/minimum (+/-) for the rotation

    endExpConst = 1.44 # don't change this unless you've really looked over the math

    # Set a deadband for the raw joystick input
    yDeadband = 0.08
    rDeadband = 0.05

    # Set a turning correction to help the robot drive straight (should be greater than the deadband)
    fwdTurningCorrection = 0
    revTurningCorrection = 0

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    leftMtrBaseCmd = int(10)
    rightMtrBaseCmd = int(10)

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
## @brief  Function to compute the manual arm drive command
##         following a linear control curve
## @param  aIn - raw input from -1.0 to 1.0
## @return the arm command (0 to 254)
############################################################
def manualArmLinDrive(aIn):
    deadband = 0.01
    if (-deadband < aIn and aIn < deadband):
        aIn = 0.0

    return int(interp(aIn, [-1, 1], [0, 254]))


############################################################
## @brief  Function to compute the manual arm drive command
##         following an exponential control curve
## @param  aIn - raw input from -1.0 to 1.0, where 1.0 is "up" and -1.0 is "down"
## @param  armTurbo - button which when held, gives more power to the arm
## @param  armDownSlowBtn - button which slowly drives the arm down
## @param  armUpSlowBtn - button which slowly drives the arm up
## @return the arm command (0 to 254), where 0 is "max down" and 254 is "max up"
############################################################
def manualArmExpDrive(aIn, armTurbo, armDownSlowBtn, armUpSlowBtn):

    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential function
    # See a plot at https://www.wolframalpha.com/input?i=plot+e%5E%28%28x%5E3.0%29%2F1.44%29-1+for+x+%3D+0+to+x+%3D+1
    endExpConst = 1.44 # don't change this unless you've really looked over the math
    expConst = 3.0  # exponential growth coefficient -- should be between 1.0-4.0
    downEndpoint = 95  # maximum absolute value for the arm motor command in forward
    upEndpoint = 127  # maximum absolute value for the arm motor command in reverse
    nonTurboMultiplier = 0.8  # multiplier applied when *not* in turbo mode
    armDownSlowCmd = 15  # absolute value for the arm motor in "slow" mode
    armUpSlowCmd = 25  # absolute value for the arm motor in "slow" mode

    # Set a deadband for the raw joystick input
    deadband = 0.0

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    baseCmd = int(6)

    # Apply the deadband
    if abs(aIn) < deadband:
        aIn = 0

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if aIn < 0 or armDownSlowBtn:
        neg = -1
        endpoint = downEndpoint
    else:
        neg = 1
        endpoint = upEndpoint
    
    # Compute the motor command using the exponential function (zero-based)
    aCmd = \
      int( \
        ( \
          math.pow( \
            math.e, \
            math.pow(math.fabs(aIn), expConst) / endExpConst \
          ) \
          - 1 \
        ) \
        * (endpoint - baseCmd) \
      )

    # Add in the base command if the arm command is non-zero
    if aCmd > 0:
      aCmd = aCmd + baseCmd

    # If the arm is not in turbo, then scale down the power
    if not armTurbo:
        aCmd = int(aCmd * nonTurboMultiplier)

    # The buttons override what was computed via the analog input
    if armDownSlowBtn:
        aCmd = armDownSlowCmd
    if armUpSlowBtn:
        aCmd = armUpSlowCmd

    # Re-apply the negative-ness
    aCmd = neg * aCmd

    # If the command is greater than the maximum or less than the minimum, scale it back
    if aCmd > maxCommand:
        aCmd = maxCommand
    elif aCmd < minCommand:
        aCmd = minCommand

    # Shift the command to be based on the zeroCommand (above)
    aCmd = aCmd + zeroCommand

    return aCmd

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

    for i in range (0, 3):
        ser.write((255).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))
        ser.write((127).to_bytes(1, byteorder='big'))


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


if __name__ == '__main__':
    sys.exit(int(main() or 0))
