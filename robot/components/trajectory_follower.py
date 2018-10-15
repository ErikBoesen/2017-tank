from typing import Tuple
import wpilib
from wpilib import drive
import pathfinder as pf
from robotpy_ext.common_drivers import navx


class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """

    #TODO FIND THE REAL VALUES
    WHEEL_DIAMETER = 6
    KV = 1.101
    KA = 0.102

    drivetrain: drive.DifferentialDrive
    navx: navx.AHRS
    l_encoder: wpilib.Encoder
    r_encoder: wpilib.Encoder

    def on_enable(self):
        self.right_max = 0
        self.right_min = 0
        self.left_max = 0
        self.left_min = 0
        self.last_difference = 0

        self.left_follower = pf.followers.EncoderFollower(None)
        self.right_follower = pf.followers.EncoderFollower(None)

        self.left_follower.configurePIDVA(1.0, 0, 0, self.KV, self.KA)
        self.right_follower.configurePIDVA(1.0, 0, 0, self.KV, self.KA)

        self.left_follower.configureEncoder(self.l_encoder.get(), 1000, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(self.r_encoder.get(), 1000, self.WHEEL_DIAMETER)

    def follow_trajectory(self, trajectory: Tuple[pf.Segment, pf.Segment]):
        self.left_follower.setTrajectory(trajectory[0])
        self.right_follower.setTrajectory(trajectory[1])

        self.left_follower.configureEncoder(self.l_encoder.get(), 1000, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(self.r_encoder.get(), 1000, self.WHEEL_DIAMETER)

    def execute(self):
        if self.left_follower.trajectory is None or self.right_follower.trajectory is None:
            return

        if self.left_follower.isFinished() and self.right_follower.isFinished():
            self.drivetrain.stopMotor()
            return

        left_output = self.left_follower.calculate(self.l_encoder.get())
        right_output = self.right_follower.calculate(self.r_encoder.get())

        gyro_heading = -self.navx.getAngle()  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(self.left_follower.getHeading())  # Should also be in degrees

        # This is a PD controller
        angle_difference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = (9.5 * (-1.0 / 80.0) * angle_difference) + (0.7 * (angle_difference - self.last_difference))
        self.last_difference = angle_difference

        left_output += turn
        right_output -= turn

        self.right_max = right_output if right_output > self.right_max else self.right_max
        self.left_max = left_output if left_output > self.left_max else self.left_max

        self.right_min = right_output if right_output < self.right_min else self.right_min
        self.left_min = left_output if left_output < self.left_min else self.left_min

        left_output = self.scale(left_output, (self.left_min, self.left_max), (-1.0, 1.0))
        right_output = self.scale(right_output, (self.right_min, self.right_max), (-1.0, 1.0))

        self.drivetrain.tankDrive(left_output, right_output)

    def scale(self, val, src, dst):
        """
        Scale the given value from the scale of src to the scale of dst.
        """
        return ((val - src[0]) / (src[1] - src[0])) * (dst[1] - dst[0]) + dst[0]




