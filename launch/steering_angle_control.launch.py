from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='larr_loader_project',
            executable='steering_angle_control',  
            name='steering_angle_control',        
            output='screen',
            parameters=[{
                'k_p': 0.05, # P gain 
                'k_i': 0.0, # I gain
                'k_d': 0.01, # D gain
                'u_m': -0.80, # maximum value of the normalized steering valve control
                'u_M': 0.80, # minimum value of the normalized steering valve control
                'steering_rate_m': -20.0, # maximum value of the steering angle [deg]
                'steering_rate_M': 20.0, # minimum value of the steering angle [deg]
                'a_u_to_valve': 5000.0, 
                'b_u_to_valve': 5000.0, # actual steering value control = a_u_to_valve * u_ + b_u_to_valve
                'target_electric_steering_actuation_control_mode': 0,
                'sim_flag': True, # simulation flag 
                'tau_sim': 0.50 # time-delay of the steering rate (only for simulation)
                }],  
        ),
    ])
