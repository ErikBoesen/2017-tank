from typing import Tuple, List
import wpilib
from wpilib import drive
import pathfinder as pf
from robotpy_ext.common_drivers import navx
from magicbot.magic_reset import will_reset_to


class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """
    # TODO FIND THE REAL VALUES
    WHEEL_DIAMETER = 0.5
    KV = 1.101
    KA = 0.225 # 102

    drivetrain: drive.DifferentialDrive
    navx: navx.AHRS
    l_encoder: wpilib.Encoder
    r_encoder: wpilib.Encoder

    def on_enable(self):
        self.last_difference = 0

        self.left_follower = pf.followers.EncoderFollower(None)
        self.right_follower = pf.followers.EncoderFollower(None)

        self.left_follower.configurePIDVA(1.0, 0, 0, 1 / 5, 0)
        self.right_follower.configurePIDVA(1.0, 0, 0, 1 / 5, 0)

        self._cofigure_encoders()

    def follow_trajectory(self, trajectories: Tuple[List[pf.Segment], List[pf.Segment]]):
        self.left_follower.setTrajectory(trajectories[0])
        self.right_follower.setTrajectory(trajectories[1])

        self._cofigure_encoders()

    def _cofigure_encoders(self):
        self.left_follower.configureEncoder(self.l_encoder.get(), 360, self.WHEEL_DIAMETER)
        self.right_follower.configureEncoder(self.r_encoder.get(), 360, self.WHEEL_DIAMETER)

    def execute(self):
        if (self.left_follower.trajectory is None or self.right_follower.trajectory is None) or \
           (self.left_follower.isFinished() and self.right_follower.isFinished()):
           return

        l = self.left_follower.calculate(self.l_encoder.get())
        r = self.right_follower.calculate(self.r_encoder.get())

        gyro_heading = (
            -self.navx.getAngle()
        )  # Assuming the gyro is giving a value in degrees
        desired_heading = pf.r2d(
            self.left_follower.getHeading()
        )  # Should also be in degrees

        # This is a poor man's P controller
        angle_difference = pf.boundHalfDegrees(desired_heading - gyro_heading)
        turn = (5 * (-1.0 / 80.0) * angle_difference) + (0.1 * (angle_difference - self.last_difference))
        # turn = 5 * (-1.0 / 80.0) * angleDifference
        # turn = 0

        self.last_difference = angle_difference

        l = l + turn
        r = r - turn

        # -1 is forward, so invert both values
        self.drivetrain.tankDrive(-l, -r)
