# Launch file for m2
import yaml
import os
import shutil
from launch import LaunchDescription
from launch_ros.actions import  Node
from launch_ros.actions.node import ExecuteProcess
from datetime import datetime
from ament_index_python.packages import get_package_share_directory

#### PATH TO MODAQ CONFIG FILE ######
## FYI THIS PULLS THE FILE FROM THE INSTALL DIRECTORY, NOT THE SRC DIRECTORY
configName = 'm2_config.yaml'
config = os.path.join(
      get_package_share_directory('m2_launch'),
      'config',
      configName
      )

cfg = yaml.safe_load(open(config))
logPath = cfg["M2Supervisor"]["ros__parameters"]["loggerPath"]

systemStartTime = datetime.utcnow().strftime("%Y_%m_%d_%H_%M_%S")
## Copy config file for later review
shutil.copy(config, logPath+"/"+systemStartTime+"_"+configName)

def generate_launch_description():

  return LaunchDescription([

    Node(
        name = "AdnavCompass",
        package = 'adnav_driver',
        executable = 'adnav_driver',
        emulate_tty = True,
        output = 'screen',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters = [config]
    ),
    Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='XsensINS',
            output='screen',
            parameters=[config],
            arguments=[]
            ),
    Node(
    package    = "bag_recorder",
    executable = "bag_recorder_node",
    ## Automatically gets named BagRecorded - do not add name= ""
    parameters = [config],
    output="screen",
    respawn = True
    )
  ]
                           
   )

