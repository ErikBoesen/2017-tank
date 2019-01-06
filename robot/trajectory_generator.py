import os
import pickle
import pathfinder as pf
import wpilib


WHEELBASE_WIDTH = 2 # In feet
TRAJECTORY_DIRECTORY = 'trajectories'


trajectories = {
    "charge": [
        pf.Waypoint(0, 0, 0), # Waypoints are relative to first, so start at 0, 0, 0
        pf.Waypoint(10, 0, 0)
    ],
    "diagonal": [
        pf.Waypoint(0, 0, 0),
        pf.Waypoint(15, 5, 0)
    ]
}

def load_trajectories():
    if wpilib.RobotBase.isSimulation():
        generated_trajectories = _generate_trajectories()
        _write_trajectories(generated_trajectories)
    else:
        with open(os.path.join(os.path.dirname(__file__) + os.sep + TRAJECTORY_DIRECTORY, 'trajectories.pickle'), 'rb') as f:
            generated_trajectories = pickle.load(f)

    return generated_trajectories

def _write_trajectories(trajectories):
    pickle_file = os.path.join(os.path.dirname(__file__) + os.sep + TRAJECTORY_DIRECTORY, 'trajectories.pickle')
    with open(pickle_file, 'wb') as f:
        pickle.dump(trajectories, f)

def _generate_trajectories():
    generated_trajectories = {}

    for trajectory_name in trajectories.keys():
        generated_trajectory = pf.generate(
                                   trajectories[trajectory_name],
                                   pf.FIT_HERMITE_CUBIC,
                                   pf.SAMPLES_HIGH,
                                   dt=0.02, # 20ms
                                   max_velocity=10.903,
                                   max_acceleration=53.251,
                                   max_jerk=120
                               )[1] # The 0th element is just info

        modifier = pf.modifiers.TankModifier(generated_trajectory).modify(WHEELBASE_WIDTH)

        generated_trajectories.update({
            trajectory_name: (
                modifier.getLeftTrajectory(),
                modifier.getRightTrajectory()
            )
        })

    if wpilib.RobotBase.isSimulation():
        from pyfrc.sim import get_user_renderer

        renderer = get_user_renderer()
        if renderer:
            renderer.draw_pathfinder_trajectory(modifier.getLeftTrajectory(), '#0000ff', offset=(-1, 0))
            renderer.draw_pathfinder_trajectory(modifier.source, '#00ff00', show_dt=True)
            renderer.draw_pathfinder_trajectory(modifier.getRightTrajectory(), '#0000ff', offset=(1, 0))

    return generated_trajectories
