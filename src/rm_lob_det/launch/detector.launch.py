import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取配置文件路径
    pkg_share = get_package_share_directory('detector')
    
    # 【修改点 1】：更新为你的新配置文件名 (如果你没改物理文件名，这里请写回 node_param.yaml)
    config_file = os.path.join(pkg_share, 'config', 'lob_vision_params.yaml')

    return LaunchDescription([
        # 【修改点 2】：启动我们融合后的飞镖视觉中枢节点
        Node(
            package='detector',
            executable='lob_vision_node_exec',  # <--- 【核心关键】：必须带 _exec 后缀！
            name='lob_vision_node',             # <--- 节点名称，必须和 yaml 里的顶层名字完全一致
            output='screen',
            emulate_tty=True,
            parameters=[config_file]
        )
    ])