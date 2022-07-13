from rocket_state_controller import MetaDataDistributor
from motion_to_ground import GroundSpeeds
import numpy as np

if __name__ == '__main__':
  mdd = MetaDataDistributor()
  gs = GroundSpeeds(mdd)
  uvw = np.array([1,1,1])
  pqr = np.array([1,1,1])
  t_hb = np.array([1,1,1])
  t_sb = np.array([1,1,1])
  xyz_s = np.array([1,1,1])

  uvw = mdd.set_var({'uvw': uvw})
  pqr = mdd.set_var({'pqr': pqr})
  t_hb = mdd.set_var({'t_hb': t_hb})
  t_sb = mdd.set_var({'t_sb': t_sb})
  xyz_s = mdd.set_var({'xyz_s': xyz_s})

  print(mdd.get('uvw'))
