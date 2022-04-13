# TO-DO: Containerize to microservice
import numpy as np

# This microservice computes the motion vectors of
# the flight body relative to the ground.
# Requires: Direction cosines
# Required by: Motion relative to air
# Outputs: Ground speeds in different coordinate systems, flight path angle, slip angle, and euler angles
class GroundSpeeds():
  def __init__(self, metadata_distributor):
    self.metadata_distributor = metadata_distributor
    self.constants = self.metadata_distributor.constants
    self.uvw = self.metadata_distributor.get_var('uvw')
    self.pqr = self.metadata_distributor.get_var('pqr')
    self.t_hb = self.metadata_distributor.get_var('t_hb')
    self.t_sb = self.metadata_distributor.get_var('t_sb')
    self.xyz_s = self.metadata_distributor.get_var('xyz_s')

    def get_ecef_vel(self):
      x_s, y_s, z_s = self.xyz_s
      uvw_g = self.uvw - np.matmul(self.t_sb, [-y_s*self.constants.OMEGA, x_s*self.constants.OMEGA, 0])
      self.metadata_distributor.set({'uvw_g': uvw_g})
      return uvw_g

    def get_ecef_ang_v(self):
      pqr_g = self.pqr - np.matmul(self.t_sb, [0,0,self.constants.OMEGA])
      self.metadata_distributor.set({'pqr_g': pqr_g})
      return self.pqr_g

    def get_euler_angles(self):
      if (1-self.t_hb[1,3] > 2e-4):
        phi = np.arctan(self.t_hb[2,3]/self.t_hb[3,3])
        theta = np.arcsin(-self.t_hb[1,3])
        sai = np.arctan(-self.t_hb[1,2],self.t_hb[1,1])
      else:
        phi = 0
        theta = np.arcsin(-self.t_hb[1,3])
        sai = np.arctan(-self.t_hb[2,1],self.t_hb[2,2])
      self.metadata_distributor.set({'phi': phi, 'theta': theta, 'sai': sai})
      return np.array([phi, theta, sai])

    def get_ned_vel_eci(self):
      uvw_h = np.matmul(self.t_hb.T, self.uvw)
      self.metadata_distributor.set({'uvw_h': uvw_h})
      return uvw_h

    def get_ned_vel_ecef(self):
      x_s, y_s, z_s = self.xyz_s
      uvw_hg = self.metadata_distributor.get_var('uvw_h')
      uvw_hg[1] -= (x_s**2+y_s**2)*self.constants.OMEGA
      self.metadata_distributor.set({'uvw_hg': uvw_hg})
      return uvw_hg

    def get_flight_path_angle(self):
      u_hg, v_hg, w_hg = self.metadata_distributor.get_var('uvw_hg')
      gamma = np.arctan(-w_hg/(u_hg**2+v_hg**2)**0.5)
      self.metadata_distributor.set({'gamma': gamma})
      return gamma

    def get_azimuth(self):
      u_hg, v_hg, w_hg = self.metadata_distributor.get_var('uvw_hg')
      xi = np.arctan(v_hg/u_hg)
      self.metadata_distributor.set({'xi': xi})
      return xi