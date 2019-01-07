import math
from pyfrc.physics import motor_cfgs, tankmodel
from pyfrc.physics.units import units


class PhysicsEngine(object):
    """
    Simulates a 4-wheel robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                       to communicate simulation effects to
        """
        self.physics_controller = physics_controller

        self.drivetrain = tankmodel.TankModel.theory(
            motor_cfgs.MOTOR_CFG_CIM,           # motor configuration
            110 * units.lbs,                      # robot mass
            12.75,                              # drivetrain gear ratio
            2,                                  # motors per side
            17.75 * units.inch,                      # robot wheelbase
            21.63 * units.inch,     # robot width
            28 * units.inch,     # robot length
            6 * units.inch                        # wheel diameter
        )

        self.kEncoder = 360 / (0.5 * math.pi)

        self.l_distance = 0
        self.r_distance = 0

    def update_sim(self, hal_data, now, tm_diff):
        """
        Called when the simulation parameters for the program need to be
        updated.

        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        lf_motor = hal_data['pwm'][9]['value']
        rf_motor = hal_data['pwm'][7]['value']

        # Not needed because front and rear should be in sync
        # lr_motor = hal_data['pwm'][8]['value']
        # rr_motor = hal_data['pwm'][6]['value']

        x, y, angle = self.drivetrain.get_distance(lf_motor, rf_motor, tm_diff)
        self.physics_controller.distance_drive(x, y, angle)

        # Update encoders
        self.l_distance += self.drivetrain.l_velocity * tm_diff
        self.r_distance += self.drivetrain.r_velocity * tm_diff

        hal_data['encoder'][0]['count'] = int(self.l_distance * self.kEncoder)
        hal_data['encoder'][1]['count'] = int(self.r_distance * self.kEncoder)
