import wpilib
from magicbot import will_reset_to
from magicbot import tunable

SPEED_MULTIPLIER = 1 / 2


class Intake:
    """
    Operate robot intake.
    """
    intake_wheels: wpilib.SpeedControllerGroup

    def __init__(self):
        self._intake_wheel_speed = will_reset_to(0)

    def spin(self, speed: float=SPEED_MULTIPLIER):
        """
        Set the speed of intake wheels.

        :param speed: The requested speed, between -1 and 1.
        """
        self._intake_wheel_speed = speed

    def pull(self):
        """
        Pull cube into intake.
        """
        self.spin(-SPEED_MULTIPLIER)

    def push(self):
        """
        Push cube out of intake.
        """
        self.spin(SPEED_MULTIPLIER)

    def execute(self):
        """
        Run intake motors.
        """
        self.intake_wheels.set(self._intake_wheel_speed)
