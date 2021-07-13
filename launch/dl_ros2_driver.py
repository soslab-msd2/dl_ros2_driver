
import os
import launch

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    dl_ros2_driver = Node(
        node_name = 'dl_ros2_driver',
        package = 'dl_ros2_driver',
        node_executable = 'dl_ros2_driver_node',
        output = 'screen',
        parameters = [
            {'dl_ip': '10.10.31.180'},
            {'dl_tcp_port': 50660},
            {'pc_port': 45454},

            {'max_distance': 15000},
            {'integration_time_low': 3000},
            {'integration_time_mid': 0},
            {'integration_time_high': 0},
            {'integration_time_grayscale': 0},
            {'hdr': 0},   # 0 = off, 1 = HDR1, 2 = HDR2
            {'modulation_frequency': 0},   # 0 = 12MHz, 1 = 24MHz, 2 = 6MHz, 3 = 3MHz, 4 = 1.5MHz, 5 = 0.75MHz
            {'modulation_channel': 1},
            {'modulation_auto_channel': 1},
            {'min_amplitude': 1},
            {'offset': 0},

            {'cam_cal_fx': 204.374692},
            {'cam_cal_fy': 206.172834},
            {'cam_cal_cx': 164.42},
            {'cam_cal_cy': 122.842145},
            {'cam_cal_k1': -0.378984},
            {'cam_cal_k2': 0.108586},
            {'cam_cal_p1': -0.001025},
            {'cam_cal_p2': 0.001098},

            {'filter_on': True},
            {'flip_rightleft': True},
            {'flip_updown': False},

            {'frame_id': 'base_link'},
            {'pub_topicname_distance_image': 'image_distance'},
            {'pub_topicname_amplitude_image': 'image_amplitude'},
            {'pub_topicname_pointcloud': 'pointcloud'},
        ],
    )

    ld = launch.LaunchDescription()
    ld.add_action( dl_ros2_driver )
    
    return ld
