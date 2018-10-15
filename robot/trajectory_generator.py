import os.path
import pickle
import pathfinder as pf
import wpilib
import math

points = [
    pf.Waypoint(2, 1, 0), # Waypoint @ x=0, y=0,   exit angle=0 radians
    pf.Waypoint(5, 0, 0)
]

info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                               dt=0.02, # 20ms
                               max_velocity=10.903,
                               max_acceleration=117.152,
                               max_jerk=300.0)

modifier = pf.modifiers.TankModifier(trajectory).modify(0.5)

left_trajectory = None
right_trajectory = None

# because of a quirk in pyfrc, this must be in a subdirectory
# or the file won't get copied over to the robot
picke_file_left = os.path.join(os.path.dirname(__file__), 'trajectory_left.pickle')
pickle_file_right = os.path.join(os.path.dirname(__file__), 'trajectory_right.pickle')

if wpilib.RobotBase.isSimulation():
    # generate the trajectory here

    left_trajectory = modifier.getLeftTrajectory()
    right_trajectory = modifier.getRightTrajectory()

    # and then write it out
    with open(picke_file_left, 'wb') as fp:
        pickle.dump(modifier.getLeftTrajectory(), fp)
    with open(pickle_file_right, 'wb') as fp:
        pickle.dump(modifier.getRightTrajectory(), fp)

    from pyfrc.sim import get_user_renderer

    renderer = get_user_renderer()
    if renderer:
        renderer.draw_pathfinder_trajectory(modifier.getLeftTrajectory(), '#0000ff', offset=(-1, 0))
        renderer.draw_pathfinder_trajectory(modifier.source, '#00ff00')
        renderer.draw_pathfinder_trajectory(modifier.getRightTrajectory(), '#0000ff', offset=(1, 0))

else:
    with open('trajectory_left.pickle', 'rb') as fp:
        left_trajectory = pickle.load(fp)
    with open('trajectory_right.pickle', 'rb') as fp:
        right_trajectory = pickle.load(fp)