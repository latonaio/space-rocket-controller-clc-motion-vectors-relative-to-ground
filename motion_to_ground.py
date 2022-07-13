# TO-DO: Containerize to microservice
import numpy as np
import logging
import asyncio
import os
from rabbitmq_client import RabbitmqClient

logger = logging.getLogger(__name__)


# This microservice computes the motion vectors of
# the flight body relative to the ground.
# Requires: Direction cosines
# Required by: Motion relative to air
# Outputs: Ground speeds in different coordinate systems, flight path angle, slip angle, and euler angles
class GroundSpeeds():
    def __init__(self, message):
        self.pqr_g = None
        self.data = message.data
        self.uvw = self.data.get('uvw')
        self.pqr = self.data.get('pqr')
        self.t_hb = self.data.get('t_hb')
        self.t_sb = self.data.get('t_sb')
        self.xyz_s = self.data.get('xyz_s')

    def get_ecef_vel(self):
        x_s, y_s, z_s = self.xyz_s
        uvw_g = self.uvw - np.matmul(self.t_sb, [-y_s * self.data.OMEGA, x_s * self.data.OMEGA, 0])
        self.data.set({'uvw_g': uvw_g})
        return uvw_g

    def get_ecef_ang_v(self):
        pqr_g = self.pqr - np.matmul(self.t_sb, [0, 0, self.data.OMEGA])
        self.data.set({'pqr_g': pqr_g})
        return pqr_g

    def get_euler_angles(self):
        if 1 - self.t_hb[1, 3] > 2e-4:
            phi = np.arctan(self.t_hb[2, 3] / self.t_hb[3, 3])
            theta = np.arcsin(-self.t_hb[1, 3])
            sai = np.arctan(-self.t_hb[1, 2], self.t_hb[1, 1])
        else:
            phi = 0
            theta = np.arcsin(-self.t_hb[1, 3])
            sai = np.arctan(-self.t_hb[2, 1], self.t_hb[2, 2])
        self.data.set({'phi': phi, 'theta': theta, 'sai': sai})
        return np.array([phi, theta, sai])

    def get_ned_vel_eci(self):
        uvw_h = np.matmul(self.t_hb.T, self.uvw)
        self.data.set({'uvw_h': uvw_h})
        return uvw_h

    def get_ned_vel_ecef(self):
        x_s, y_s, z_s = self.xyz_s
        uvw_hg = self.data.get('uvw_h')
        uvw_hg[1] -= (x_s ** 2 + y_s ** 2) * self.data.OMEGA
        self.data.set({'uvw_hg': uvw_hg})
        return uvw_hg

    def get_flight_path_angle(self):
        u_hg, v_hg, w_hg = self.data.get_var('uvw_hg')
        gamma = np.arctan(-w_hg / (u_hg ** 2 + v_hg ** 2) ** 0.5)
        self.data.set({'gamma': gamma})
        return gamma

    def get_azimuth(self):
        u_hg, v_hg, w_hg = self.data.get_var('uvw_hg')
        xi = np.arctan(v_hg / u_hg)
        self.data.set({'xi': xi})
        return xi


async def main():
    rabbitmq_url = os.environ['RABBITMQ_URL']
    queue_origin = os.environ['QUEUE_ORIGIN']
    logger.info('create mq client')
    queue_to = os.environ['QUEUE_TO']

    try:
        mq_client = await RabbitmqClient.create(rabbitmq_url, {queue_origin}, {queue_to})
    except Exception as e:
        logger.error({
            'message': 'failed to connect rabbitmq!',
            'error': str(e),
            'queue_origin': queue_origin,
            'queue_to_for_log': queue_to,
        })
        os._exit(1)

    logger.info('create mq client')

    async for message in mq_client.iterator():
        try:
            async with message.process():
                logger.info({
                    'message': 'message received',
                    'params': message.data,
                })

                gs = GroundSpeeds(message)
                print(gs)
                # await mq_client.send(queue_to, gs)
        except Exception as e:
            logger.error({
                'message': 'failed to connect rabbitmq!',
                'error': str(e),
            })


if __name__ == '__main__':
    asyncio.run(main())
