from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import EnvironmentVariable, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    acados_source = '/home/lu/.local/share/acados'

    return LaunchDescription([
        SetEnvironmentVariable('ACADOS_SOURCE_DIR', acados_source),
        SetEnvironmentVariable('ACADOS_PYTHON', f'{acados_source}/venv/bin/python'),
        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            [
                TextSubstitution(text=f'{acados_source}/lib:'),
                TextSubstitution(text=f'{acados_source}/build/acados:'),
                TextSubstitution(text=f'{acados_source}/build/external/hpipm:'),
                TextSubstitution(text=f'{acados_source}/build/external/blasfeo:'),
                EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
            ],
        ),
        SetEnvironmentVariable(
            'PYTHONPATH',
            [
                TextSubstitution(text=f'{acados_source}/interfaces/acados_template:'),
                EnvironmentVariable('PYTHONPATH', default_value=''),
            ],
        ),
        Node(
            package='multi_usv_planner',
            executable='heron_single_node',
            name='heron_single_node',
            output='screen',
            parameters=[{
                'goal.x': 40.0,
                'goal.y': 0.0,
                'use_acados': True,
            }],
        ),
    ])
