from typing import List
from pathfinder import followers, Segment

class TrajectoryFollower:
    """
    Move along generated paths for autonomous
    """

    def __init__(self):
        self.left_follower = None
        self.right_follower = None

    def follow_trajectory(self, trajectory: List[Segment]):
        self.left_follower = followers.EncoderFollower(trajectory[0])
        self.right_follower = followers.EncoderFollower(trajectory[1])




    def execute(self):
        pass
