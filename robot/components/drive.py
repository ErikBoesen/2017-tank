import wpilib

ENCODER_ROTATION = 1023
WHEEL_DIAMETER = 7.639

SARAH_MULTIPLIER = 0.5


class Drive:
    robot_drive = wpilib.RobotDrive

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
        :param y: The speed that the robot should drive in the Y direction. -1 is forward. [-1.0..1.0]
        :param rotation: The rate of rotation for the robot that is completely independent of the translation. 1 is rotate to the right [-1.0..1.0]
        :param sarah: Is Sarah driving?
        """
        if sarah:
            y *= SARAH_MULTIPLIER
            rotation *= SARAH_MULTIPLIER
        self.y = y
        self.rotation = rotation

    def execute(self):
        """Actually drive."""
        self.robot_drive.arcadeDrive(self.y, -self.rotation)

        # Prevent robot from driving by default
        self.y = 0
        self.rotation = 0
