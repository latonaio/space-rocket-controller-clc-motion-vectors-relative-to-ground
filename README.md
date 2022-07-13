# space-rocket-clc-motion-vectors-relative-to-ground

space-rocket-clc-motion-vectors-relative-to-ground は、宇宙ロケットのオンボーディング環境においてリアルタイムで対地状態量計算を行うマイクロサービスです。  

Requires: space-rocket-metadata-distributor


## 演算項目
本マイクロサービスでは、以下の項目における対地状態量計算が行われます。  

* uvw_g:機体重心の対地速度
* uvw_h:機体重心の慣性速度
* uvw_hg:機体重心の慣性速度の計算結果
* γ:飛行経路角
* ε:飛行方位角
* pqr_g:機体重心の対地角速度
* ΦθΨ:機体姿勢（オイラー角）
  
該当箇所は、 motion_to_ground.py の以下の部分です。  

```
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
```  


## アーキテクチャ
![アーキテクチャ1](pics/simulation_program_outline1-1.png)
![アーキテクチャ2](pics/simulation_program_outline1-2.png)
