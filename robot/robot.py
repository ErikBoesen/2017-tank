#!/usr/bin/env python3

import magicbot
import wpilib

from components import drive

from robotpy_ext.common_drivers import navx

ROT_COR = .145


class Bot(magicbot.MagicRobot):
    drive = drive.Drive

    def createObjects(self):
        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()

        # Joysticks
        self.joystick = wpilib.Joystick(0)

        # Drive motor controllers
        #   Dig | 0/1
        #   2^1 | Left/Right
        #   2^0 | Front/Rear
        self.lf_motor = wpilib.Victor(0b00)  # => 0
        self.lr_motor = wpilib.Victor(0b01)  # => 1
        self.rf_motor = wpilib.Victor(0b10)  # => 2
        self.rr_motor = wpilib.Victor(0b11)  # => 3
        self.robot_drive = wpilib.RobotDrive(self.lf_motor, self.lr_motor, self.rf_motor, self.rr_motor)

    def autonomous(self):
        super().autonomous()

    def disabledPeriodic(self): pass
    def disabledInit(self): pass
    def teleopInit(self): pass

    def teleopPeriodic(self):
        # Normal joysticks
        #self.drive.move(-self.joystick.getY(),self.joystick.getX())

        # Corrections for aviator joystick
        self.drive.move(-2*(self.joystick.getY()+.5), 2*(self.joystick.getX()+.5))



if __name__ == '__main__':
    wpilib.run(Bot)
