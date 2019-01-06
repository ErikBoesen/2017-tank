import os.path
import pickle
import pathfinder as pf
import wpilib

points = [
    pf.Waypoint(0, 0, 0), # Waypoints are relative to first, so start at 0, 0, 0
    pf.Waypoint(15, 5, 0),
]

info, trajectory = pf.generate(points, pf.FIT_HERMITE_CUBIC, pf.SAMPLES_HIGH,
                               dt=0.02, # 20ms
                               max_velocity=10.903,
                               max_acceleration=53.251,
                               max_jerk=120)

modifier = pf.modifiers.TankModifier(trajectory).modify(2)

left_trajectory = None
right_trajectory = None

trajectories = {}

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
        renderer.draw_pathfinder_trajectory(modifier.source, '#00ff00', show_dt=True)
        renderer.draw_pathfinder_trajectory(modifier.getRightTrajectory(), '#0000ff', offset=(1, 0))

else:
    with open('trajectory_left.pickle', 'rb') as fp:
        left_trajectory = pickle.load(fp)
    with open('trajectory_right.pickle', 'rb') as fp:
        right_trajectory = pickle.load(fp)
