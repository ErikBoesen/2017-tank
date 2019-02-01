#!/usr/bin/env python3

import magicbot
import wpilib

from robotpy_ext.control.button_debouncer import ButtonDebouncer
from wpilib.buttons import JoystickButton
from components import drive, intake
import wpilib.drive

ROT_COR = -0.145


class Bot(magicbot.MagicRobot):
    drive: drive.Drive
    intake: intake.Intake

    def createObjects(self):
        # Joysticks
        self.joystick = wpilib.Joystick(0)

        # Drive motor controllers
        #   Dig | 0/1
        #   2^1 | Left/Right
        #   2^0 | Front/Rear
        self.lf_motor = wpilib.Victor(9) # =>0
        self.lr_motor = wpilib.Victor(8) # =>1
        self.rf_motor = wpilib.Victor(7) # =>2
        self.rr_motor = wpilib.Victor(6) # =>3

        self.drivetrain = wpilib.drive.DifferentialDrive(wpilib.SpeedControllerGroup(self.lf_motor, self.lr_motor),
                                                    wpilib.SpeedControllerGroup(self.rf_motor, self.rr_motor))

        self.btn_sarah = ButtonDebouncer(self.joystick, 4)
        self.sarah = False

        # Intake
        self.intake_wheel_left = wpilib.Spark(0)
        self.intake_wheel_right = wpilib.Spark(1)
        self.intake_wheels = wpilib.SpeedControllerGroup(self.intake_wheel_left,
                                                         self.intake_wheel_right)
        self.intake_wheels.setInverted(True)

        self.btn_pull = JoystickButton(self.joystick, 1)
        self.btn_push = JoystickButton(self.joystick, 2)

    def autonomous(self):
        super().autonomous()

    def disabledPeriodic(self): pass
    def disabledInit(self): pass
    def teleopInit(self): pass

    def teleopPeriodic(self):
        # Normal joysticks
        #self.drive.move(-self.joystick.getY(),self.joystick.getX())

        # Corrections for aviator joystick
        self.drive.move(-2*(self.joystick.getY()+.5),
                        2*(self.joystick.getX()+.5)+ROT_COR,
                        sarah=self.sarah)

        if self.btn_sarah:
            self.sarah = not self.sarah

        if self.btn_pull.get():
            self.intake.pull()
        elif self.btn_push.get():
            self.intake.push()


if __name__ == '__main__':
    wpilib.run(Bot)
