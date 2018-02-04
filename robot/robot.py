#!/usr/bin/env python3

import magicbot
import wpilib

from networktables.util import ntproperty
from components import drive

from robotpy_ext.common_drivers import navx

from networktables.networktable import NetworkTable

class Bot(magicbot.MagicRobot):
    drive = drive.Drive

    def createObjects(self):
        # NavX (purple board on top of the RoboRIO)
        self.navx = navx.AHRS.create_spi()

        # Joysticks
        self.left_joystick = wpilib.Joystick(0)
        self.right_joystick = wpilib.Joystick(1)

        self.lf_motor = wpilib.Victor(0)
        self.lr_motor = wpilib.Victor(1)
        self.rf_motor = wpilib.Victor(2)
        self.rr_motor = wpilib.Victor(3)
        self.l = wpilib.speedControllerGroup(self.lf_motor, self.lr_motor)
        self.r = wpilib.speedControllerGroup(self.rf_motor, self.rr_motor)
        self.robot_drive = wpilib.DifferentialDrive(self.l, self.r)

    def autonomous(self): pass
    def disabledPeriodic(self): pass
    def disabledInit(self): pass
    def teleopInit(self): pass

    def teleopPeriodic(self):
        self.drive.move(-self.left_joystick.getY(), self.right_joystick.getX())


if __name__ == '__main__':
    wpilib.run(Bot)
