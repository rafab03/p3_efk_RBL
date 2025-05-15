
import rclpy

import numpy as np

from .motion_models.acceleration_motion_models import acceleration_motion_model_linearized_1 
from .observation_models.odometry_imu_observation_models import odometry_imu_observation_model_with_acceleration_motion_model_linearized_1

from .filters.ekf import ExtendedKalmanFilter
from .kf_node import KalmanFilterFusionNode as ExtendedKalmanFilterFusionNode



def main(args=None):
    # Initialize the Kalman Filter

    mu0 = np.zeros(7)
    Sigma0 = np.eye(7)
    # TO ADJUST
    noise_config = 'ruido_alto_medida'  

    if noise_config == 'ruido_alto_proceso':
        print("Configuración de ruido: bajo (R y Q pequeñas)")
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
        obs_noise_std = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

    elif noise_config == 'ruido_alto_medida':
        print("Configuración de ruido: alto en la medición (Q grande)")
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]  # proceso bajo
        obs_noise_std = [1000.0, 1000.0, 10000.0, 60.853891945200942e-06, 10.0966227112321507e-06, 0.15387262937311438, 0.15387262937311438]   # medición alta

    elif noise_config == 'ruido_alto_proceso':
        print("Configuración de ruido: alto en el proceso (R grande)")
        proc_noise_std = [1, 1, 0.5, 1, 1, 1, 1]  # proceso alto
        obs_noise_std  = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]     # medición baja

    else:
        print(
            f"Valor desconocido para 'noise_config': '{noise_config}'. "
            "Usando configuración por defecto: ruido_bajo."
        )
        proc_noise_std = [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
        obs_noise_std = [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

    # Replace the None below with proper motion and observation model functions ===
    ekf = ExtendedKalmanFilter(mu0, Sigma0,
                               acceleration_motion_model_linearized_1,  # TODO: motion_model_7d
                               odometry_imu_observation_model_with_acceleration_motion_model_linearized_1,  # TODO: observation_model_7d
                               proc_noise_std=proc_noise_std,
                               obs_noise_std=obs_noise_std)
    # ===================================================================

    rclpy.init(args=args)
    kalman_filter_node = ExtendedKalmanFilterFusionNode(ekf)
    rclpy.spin(kalman_filter_node)
    kalman_filter_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
