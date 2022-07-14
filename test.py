from rocket_state_controller import MetaDataDistributor
from motion_to_ground import GroundSpeeds
import numpy as np


if __name__ == '__main__':
    mdd = MetaDataDistributor()
    uvw = np.array([1, 1, 1])
    pqr = np.array([1, 1, 1])
    t_hb = np.ones([3, 3])
    t_sb = np.ones([3, 3])
    xyz_s = np.array([1, 1, 1])

    mdd.set({'uvw': uvw})
    mdd.set({'pqr': pqr})
    mdd.set({'t_hb': t_hb})
    mdd.set({'t_sb': t_sb})
    mdd.set({'xyz_s': xyz_s})

    gs = GroundSpeeds(mdd)

    print('uvw_g:', gs.get_ecef_vel())
    print('uvw_h:', gs.get_ned_vel_eci())
    print('uvw_hg:', gs.get_ned_vel_ecef())
    print('pqr_g:', gs.get_ecef_ang_v())
    print('angles:', gs.get_euler_angles())
    print('gamma:', gs.get_flight_path_angle())
    print('xi:', gs.get_azimuth())
