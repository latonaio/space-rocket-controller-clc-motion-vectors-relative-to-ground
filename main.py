import numpy as np
import logging
import asyncio
import os
import json
from rabbitmq_client import RabbitmqClient
from npencoder import NpEncoder
from motion_to_ground import GroundSpeeds
from rocket_state_controller import MetaDataDistributor
from custom_logger import init_logger

logger = logging.getLogger(__name__)


def calculate():
    try:
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

        return {
            'uvw': {
                'uvw_g': gs.get_ecef_vel(),
                'uvw_h': gs.get_ned_vel_eci(),
                'uvw_hg': gs.get_ned_vel_ecef()
            },
            'pqr': {
                'pqr_g': gs.get_ecef_ang_v(),
            },
            'angles': gs.get_euler_angles(),
            'gamma': gs.get_flight_path_angle(),
            'xi': gs.get_azimuth()
        }
    except Exception as e:
        logger.error({
            'message': 'calculation error',
            'error': str(e),
        })


async def main():
    init_logger()

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
            'queue_to': queue_to,
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

                result = calculate()

                await mq_client.send(queue_to, json.dumps(result, cls=NpEncoder))
        except Exception as e:
            logger.error({
                'message': 'failed to connect rabbitmq!',
                'error': str(e),
            })


if __name__ == '__main__':
    asyncio.run(main())
