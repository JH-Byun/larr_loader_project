from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share_directory = get_package_share_directory('larr_loader_project') # Get the path to the package's share directory
    rviz_config_path = package_share_directory + '/rviz/rviz_basic_settings.rviz' # Get the path to the rviz directory
    return LaunchDescription([
        Node(
            package='larr_loader_project',
            executable='loader_sim',  
            name='loader_sim',        
            parameters=[{
                'tau_': 0.10, # time-delay constant for the longitudinal acceleration tracking
                'p_sagps_r_x': -0.50, # displacement from steering point to rear gps's position (x-axis element)
                'p_sagps_r_y': 0.0, # displacement from steering point to rear gps's position (y-axis element)
                'p_sagps_r_z': 0.0, # displacement from steering point to rear gps's position (z-axis element)
                'p_sagps_f_x': 0.20, # displacement from steering point to front gps's position (x-axis element)
                'p_sagps_f_y': 0.0, # displacement from steering point to front gps's position (y-axis element)
                'p_sagps_f_z': 0.0 # displacement from steering point to front gps's position (z-axis element)
                }],  
            output='screen',
        ),

        # Launch RViz2 with a custom configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
