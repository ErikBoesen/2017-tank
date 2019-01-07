from magicbot.state_machine import state, timed_state, AutonomousStateMachine
# from automations import
# from magicbot import tunable
from components import drive, trajectory_follower


class Charge(AutonomousStateMachine):
    MODE_NAME = 'Charge'
    DEFAULT = True

    drive: drive.Drive
    follower: trajectory_follower.TrajectoryFollower

    @state(first=True)
    def charge(self, initial_call):
        if initial_call:
            self.follower.follow_trajectory('charge')

        if not self.follower.is_following('charge'):
            self.done()  # If using mutliple states use self.next_state(name)
