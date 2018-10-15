from magicbot.state_machine import state, timed_state, AutonomousStateMachine
# from automations import
# from magicbot import tunable
from components import drive, trajectory_follower
from trajectory_generator import left_trajectory, right_trajectory


class Charge(AutonomousStateMachine):
    MODE_NAME = 'Charge'
    DEFAULT = True

    drive = drive.Drive
    follower: trajectory_follower.TrajectoryFollower

    @timed_state(duration=3)
    def charge(self):
        # Move forward
        self.drive.move(1, 0)

    @state(first=True)
    def trajectory_follower(self, initial_call):
        if initial_call:
            self.follower.follow_trajectory((left_trajectory, right_trajectory))
