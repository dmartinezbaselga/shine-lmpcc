import numpy as np

# Some collision region classes

"""
Class that describes a set of discs as collision region
"""
class Discs:

    def __init__(self, n_discs, width=-1., length=-1., center_offset=0., radius=-1.):

        if n_discs <= 0:
            raise IOError("Trying to create a collision region with less than a disc")
        elif n_discs == 1:
            if width != -1:
                self.init_single_disc(width / 2.) # Ignores offset...
            elif radius != -1:
                self.init_single_disc(radius)
            else:
                raise IOError("Either the width or radius of the disc has to be specified!")
        else:

            self.radius = width / 2.
            self.offsets = np.zeros((n_discs, 1), dtype=float)

            for disc_it in range(n_discs):

                if disc_it == 0:
                    self.offsets[disc_it] = -center_offset + self.radius  # position first disc so it touches the back of the car
                elif disc_it == n_discs - 1:
                    self.offsets[disc_it] = -center_offset + length - self.radius
                else:
                    self.offsets[disc_it] = -center_offset + self.radius + disc_it * (length - 2. * self.radius) / (n_discs - 1.)
        # print(self.offsets)
        # print(self.radius)
        # print(self.offsets[-1] + self.radius)

    def init_single_disc(self, radius):
        self.offsets = [0.]
        self.radius = radius


#  Classes that hold system properties
#  - Upper and lower bound on state and inputs
#  - System name

class Prius:

    def __str__(self):
        return self.name

    def __init__(self):
        '''
        Real Prius bounds (with safety margins)
        '''
        self.name = 'Prius'
        self.init_dimensions()

        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -2000.0
        self.upper_bound['x'] = 2000.0

        self.lower_bound['y'] = -2000.0
        self.upper_bound['y'] = 2000.0

        self.lower_bound['psi'] = -np.pi
        self.upper_bound['psi'] = +np.pi

        self.lower_bound['v'] = -2.0
        self.upper_bound['v'] = 6. # Limited for safety

        self.lower_bound['delta'] = -0.45 #-1.22 # These limits do not seem to come from the Prius?
        self.upper_bound['delta'] = 0.45 #1.22

        self.lower_bound['delta_in'] = -0.45
        self.upper_bound['delta_in'] = 0.45
        self.lower_bound['delta_in2'] = -0.45
        self.upper_bound['delta_in2'] = 0.45

        self.lower_bound['spline'] = -1.0
        self.upper_bound['spline'] = 2000.0

        # self.lower_bound['w'] = -0.4 # ORIGINAL BOUNDS
        # self.upper_bound['w'] = 0.4
        self.lower_bound['w'] = -0.2
        self.upper_bound['w'] = 0.2

        self.lower_bound['a'] = -0.45 # REAL PRIUS
        self.upper_bound['a'] = 0.45
        # self.lower_bound['a'] = -1.0 # 
        # self.upper_bound['a'] = 1.0
        #self.lower_bound['a'] = -6.0 # CARLA PRIUS
        #self.upper_bound['a'] = 2.0

        self.lower_bound['ay'] = -2.0
        self.upper_bound['ay'] = 2.0

        self.lower_bound['j'] = -5.0
        self.upper_bound['j'] = 5.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 5000

    def init_dimensions(self, carla_prius=False):

        if carla_prius:         # carla dimensions: 4.5135, 2.0
            self.L = 4.5134
            cog_from_center = 0.5
            self.lr = self.L / 2. + cog_from_center
            self.lf = self.L - self.lr

        else:
            self.lr = 1.577
            self.lf = 1.123
            self.L = self.lr + self.lf
            # lr = 1.38 (these are older prius values I think)
            # lf = 1.61
            # L = lr + lf

            self.length = 4.54
            self.width = 2.25
            self.com_to_back = 2.4

        self.ratio = self.lr/(self.lr + self.lf)
        self.area = Discs(3, width=self.width, length=self.length, center_offset=self.com_to_back)


class Carla:

    def __str__(self):
        return self.name

    def __init__(self):
        '''
        Simulated vehicle bounds (different from real system for more aggressive simulations)
        '''
        self.name = 'Carla'
        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -2000.0
        self.upper_bound['x'] = 2000.0

        self.lower_bound['y'] = -2000.0
        self.upper_bound['y'] = 2000.0

        self.lower_bound['psi'] = -np.pi
        self.upper_bound['psi'] = +np.pi

        self.lower_bound['v'] = -0.01
        self.upper_bound['v'] = 8.0

        self.lower_bound['delta'] = -0.9 #-1.22 # -0.9, 0.9
        self.upper_bound['delta'] = 0.9 #1.22

        self.lower_bound['spline'] = -1.0
        self.upper_bound['spline'] = 10000.0

        self.lower_bound['w'] = -0.8 #-1.2 also seems too high  # 2.0 seems to high
        self.upper_bound['w'] = 0.8 #0.4 #1.2

        self.lower_bound['alpha'] = -1.0  # Not correct!
        self.upper_bound['alpha'] = 1.0

        self.lower_bound['a'] = -6.0 # Limited because of low-level control #-8.0 -> 0.45 for prius (make configurable)
        self.upper_bound['a'] = 3.0 # Limited because of low-level control # #3.0

        self.lower_bound['ay'] = -2.0
        self.upper_bound['ay'] = 2.0

        self.lower_bound['j'] = -3.0
        self.upper_bound['j'] = 3.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 5000


class Jackal:

    def __str__(self):
        return str(type(self))

    def __init__(self):
        self.name = 'Jackal'

        self.length = 0.5
        self.width = 0.5
        self.com_to_back = self.length / 2.
        self.area = Discs(1, width=self.width)

        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -2000.0
        self.upper_bound['x'] = 2000.0

        self.lower_bound['y'] = -2000.0
        self.upper_bound['y'] = 2000.0

        self.lower_bound['psi'] = -2*np.pi
        self.upper_bound['psi'] = +2*np.pi

        self.lower_bound['v'] = 0.00
        self.upper_bound['v'] = 1.5 # 2.0 in jackal simulator?

        self.lower_bound['spline'] = -1.0
        self.upper_bound['spline'] = 10000.0

        self.lower_bound['w'] = -1.1 # 20-2 lowered from 1.0
        self.upper_bound['w'] = 1.1 # 20-2 lowered from 1.0

        self.lower_bound['alpha'] = -3.0  # Not correct!
        self.upper_bound['alpha'] = 3.0

        self.lower_bound['a'] = -2.0 #-8.0
        self.upper_bound['a'] = 2.0 #5.0 # 5.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 5000


class Roboat:

    def __str__(self):
        return self.name

    def __init__(self):
        self.name = 'Roboat'
        self.lower_bound = dict()
        self.upper_bound = dict()

        self.lower_bound['x'] = -2000.0
        self.upper_bound['x'] = 2000.0

        self.lower_bound['y'] = -2000.0
        self.upper_bound['y'] = 2000.0

        self.lower_bound['psi'] = -np.pi
        self.upper_bound['psi'] = +np.pi

        self.lower_bound['u'] = -1.67
        self.upper_bound['u'] = 1.67  # m/s --> 6 km/h is the velocity limit in the canals

        self.lower_bound['v'] = -1.67
        self.upper_bound['v'] = 1.67

        self.lower_bound['r'] = -2.0
        self.upper_bound['r'] = 2.0

        self.lower_bound['spline'] = -1.0
        self.upper_bound['spline'] = 10000.0

        self.lower_bound['f1'] = -6.0  # 1630 N for the full scale, 6.0 N for the quarterscale Roboats
        self.upper_bound['f1'] = 6.0

        self.lower_bound['f2'] = -6.0
        self.upper_bound['f2'] = 6.0

        self.lower_bound['f3'] = -6.0  # 300 N for the full scale, 6.0 N for the quarterscale Roboats
        self.upper_bound['f3'] = 6.0

        self.lower_bound['f4'] = -6.0
        self.upper_bound['f4'] = 6.0

        self.lower_bound['slack'] = 0
        self.upper_bound['slack'] = 5000


class Hovergames:

    def __str__(self):
        return self.name

    def __init__(self):
        self.name = 'Hovergames'
        self.lower_bound = dict()
        self.upper_bound = dict()

        # Maximum roll and pitch angle
        max_angle = 50  # deg
        max_angle_cmd = max_angle - 10  # deg

        max_yawrate = 15  # deg/s
        max_yawrate_cmd = max_yawrate - 5  # deg/s

        self.lower_bound['x'] = -np.inf
        self.upper_bound['x'] = np.inf

        self.lower_bound['y'] = -np.inf
        self.upper_bound['y'] = np.inf

        self.lower_bound['z'] = -np.inf
        self.upper_bound['z'] = np.inf

        self.lower_bound['vx'] = -10.0
        self.upper_bound['vx'] = 10.0

        self.lower_bound['vy'] = -10.0
        self.upper_bound['vy'] = 10.0

        self.lower_bound['vz'] = -10.0
        self.upper_bound['vz'] = 10.0

        self.lower_bound['phi'] = -np.pi / 180 * max_angle
        self.upper_bound['phi'] = np.pi / 180 * max_angle

        self.lower_bound['theta'] = -np.pi / 180 * max_angle
        self.upper_bound['theta'] = np.pi / 180 * max_angle

        self.lower_bound['psi'] = -np.pi / 180 * max_angle
        self.upper_bound['psi'] = np.pi / 180 * max_angle

        self.lower_bound['spline'] = -1.0
        self.upper_bound['spline'] = 10000.0

        self.lower_bound['phi_c'] = -np.pi / 180 * max_angle_cmd
        self.upper_bound['phi_c'] = np.pi / 180 * max_angle_cmd

        self.lower_bound['theta_c'] = -np.pi / 180 * max_angle_cmd
        self.upper_bound['theta_c'] = np.pi / 180 * max_angle_cmd

        self.lower_bound['dpsi_c'] = -np.pi / 180 * max_yawrate_cmd
        self.upper_bound['dpsi_c'] = np.pi / 180 * max_yawrate_cmd

        self.lower_bound['thrust_c'] = 5
        self.upper_bound['thrust_c'] = 15
