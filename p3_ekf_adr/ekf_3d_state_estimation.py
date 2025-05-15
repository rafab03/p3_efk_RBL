
import numpy as np
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion


from .motion_models.velocity_motion_models import velocity_motion_model_linearized
from .observation_models.odometry_observation_models import odometry_observation_model_linearized

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterNode as ExtendedKalmanFilterNode


def main(args=None):
    # Initialize the Kalman Filter
    mu0 = np.zeros(3)
    Sigma0 = np.eye(3)

    noise_config = 'ruido_alto_proceso'  

    if noise_config == 'caso_base':
        print("Configuración de ruido: bajo (R y Q pequeñas)")
        proc_noise_std = [0.002, 0.002, 0.001]
        obs_noise_std = [1.02, 1.02, 100.01]

    elif noise_config == 'ruido_alto_medida':
        print("Configuración de ruido: alto en la medición (Q grande)")
        proc_noise_std = [0.002, 0.002, 0.001]  # proceso bajo
        obs_noise_std = [10.2, 10.2, 1000.1]    # medición alta

    elif noise_config == 'ruido_alto_proceso':
        print("Configuración de ruido: alto en el proceso (R grande)")
        proc_noise_std = [0.02, 0.02, 0.01]    # proceso alto
        obs_noise_std  = [1.02, 1.02, 100.01]    # medición baja

    else:
        print(
            f"Valor desconocido para 'noise_config': '{noise_config}'. "
            "Usando configuración por defecto: ruido_bajo."
        )
        proc_noise_std = [0.002, 0.002, 0.001]
        obs_noise_std = [1.02, 1.02, 100.01]

    ekf = ExtendedKalmanFilter(mu0, Sigma0, 
                               velocity_motion_model_linearized,
                               odometry_observation_model_linearized,
                               proc_noise_std = proc_noise_std,
                               obs_noise_std = obs_noise_std)
    


    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterNode(ekf)
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
