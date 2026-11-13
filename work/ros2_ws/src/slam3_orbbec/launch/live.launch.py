import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    root = os.environ['ORB_SLAM3_ROOT']
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('orbbec_camera'), 'launch', 'gemini2L.launch.py')),
        condition=IfCondition(LaunchConfiguration('start_camera')),
        launch_arguments={
            'enable_color': 'true', 'enable_depth': 'true', 'enable_ir': 'false',
            'enable_accel': 'false', 'enable_gyro': 'false',
            'depth_registration': 'true', 'enable_frame_sync': 'true',
            'enable_depth_scale': 'true', 'enable_color_undistortion': 'false',
            'enable_depth_undistortion': 'false', 'enable_point_cloud': 'false',
            'enable_colored_point_cloud': 'false', 'color_fps': '30', 'depth_fps': '30',
        }.items())
    slam = Node(package='slam3_orbbec', executable='rgbd_node', output='screen',
        sigterm_timeout='120', sigkill_timeout='5', parameters=[{
            'vocabulary': os.path.join(root, 'Vocabulary', 'ORBvoc.txt'),
            'output_dir': LaunchConfiguration('output_dir'),
            'viewer': ParameterValue(LaunchConfiguration('viewer'), value_type=bool),
            'record_dense': ParameterValue(LaunchConfiguration('record_dense'), value_type=bool),
            'rgb_topic': LaunchConfiguration('rgb_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'info_topic': LaunchConfiguration('info_topic'),
            'target_width': ParameterValue(LaunchConfiguration('target_width'),value_type=int),
            'writer_queue_size': ParameterValue(LaunchConfiguration('writer_queue_size'),value_type=int),
            'snapshot_period': ParameterValue(LaunchConfiguration('snapshot_period'),value_type=float),
            'verify_camera_params': ParameterValue(LaunchConfiguration('verify_camera_params'),value_type=bool),
            'aligned_depth_confirmed': ParameterValue(LaunchConfiguration('aligned_depth_confirmed'),value_type=bool),
            'fps': 30, 'depth_scale': 0.001,
            'max_sync_seconds': 0.02,
        }])
    dense = Node(package='slam3_orbbec',executable='dense_node.py',output='screen',
        condition=IfCondition(LaunchConfiguration('record_dense')),sigterm_timeout='240',sigkill_timeout='5',
        parameters=[{'output_dir':LaunchConfiguration('output_dir'),
                     **{k:ParameterValue(LaunchConfiguration(k),value_type=t) for k,t in
                        [('voxel',float),('stride',int),('near',float),('far',float),
                         ('outlier_radius',float),('outlier_neighbors',int),('max_voxels',int)]}}])
    rviz = Node(package='rviz2',executable='rviz2',output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d',os.path.join(get_package_share_directory('slam3_orbbec'),'rviz','dense.rviz')])
    return LaunchDescription([
        *[DeclareLaunchArgument(k,default_value=v) for k,v in [
            ('target_width','640'),('writer_queue_size','8'),('snapshot_period','2.0'),
            ('verify_camera_params','true'),('aligned_depth_confirmed','false'),
            ('voxel','0.01'),('stride','2'),('near','0.3'),('far','4.0'),
            ('outlier_radius','0.04'),('outlier_neighbors','3'),('max_voxels','600000'),('rviz','false')]],
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('record_dense', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=os.environ.get('SLAM3_RUN_DIR', 'slam3_run')),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth/image_raw'),
        DeclareLaunchArgument('info_topic', default_value='/camera/color/camera_info'),
        camera, slam, dense, rviz])
