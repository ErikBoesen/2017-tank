import wpilib
import wpilib.drive

ENCODER_ROTATION = 1023
WHEEL_DIAMETER = 7.639

SARAH_MULTIPLIER = 0.5


class Drive:
    drivetrain = wpilib.drive.DifferentialDrive

    def __init__(self):
        self.enabled = False

    def on_enable(self):
        self.y = 0
        self.rotation = 0

    # Verb functions -- these functions do NOT talk to motors directly. This
    # allows multiple callers in the loop to call our functions without
    # conflicts.

    def move(self, y, rotation, sarah=False):
        """
        Causes the robot to move
        :param y: The speed that the robot should drive in the Y direction.
        :param rotation: The rate of rotation for the robot that is completely independent of the translation.
        :param sarah: Is Sarah driving?
        """
        if sarah:
            y *= SARAH_MULTIPLIER
            rotation *= SARAH_MULTIPLIER
        self.y = y
        self.rotation = rotation

    def execute(self):
        """Actually drive."""
        self.drivetrain.arcadeDrive(self.y, self.rotation)

        # Prevent robot from driving by default
        self.y = 0
        self.rotation = 0
