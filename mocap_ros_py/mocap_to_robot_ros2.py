import json
from mocap_robotapi import *

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from docutils.parsers.rst.directives import encoding

def ros2_joint_state_publisher():       
  try:
    rclpy.init()
    node = Node("json_to_joint_state_publisher")
    publisher = node.create_publisher(JointState, "/joint_states", 10)
    timer = node.create_timer(0.1, lambda: publish_joint_state(node, publisher))
    json_file_path = './retarget.json'
    robot = MCPRobot(open(json_file_path).read())
    app = MCPApplication()
    settings = MCPSettings()
    settings.set_udp(7012)
    settings.set_bvh_rotation(0)
    app.set_settings(settings)
    app.open()
    # Joints name list
    with open(json_file_path, 'r') as file:
        data = json.load(file)
    joint_names = data['urdfJointNames']     
    
    def publish_joint_state(node, publisher):
          evts = app.poll_next_event()
          for evt in evts:
              if evt.event_type == MCPEventType.AvatarUpdated:
                  avatar = MCPAvatar(evt.event_data.avatar_handle)
                  robot.update_robot(avatar)
                  robot.run_robot_step()
                  #print (robot.get_robot_ros_frame_json())
                  
                  # get realtime data
                  real_time_data = json.loads(robot.get_robot_ros_frame_json()[0])

                  # create JointState
                  joint_state_msg = JointState()
                  joint_state_msg.header.stamp = node.get_clock().now().to_msg()
                  joint_state_msg.name = joint_names

                  # init joint positions
                  joint_positions = [0.0] * len(joint_names)

                  # fill the position
                  for i, name in enumerate(joint_names):                    
                      if name in real_time_data['joint_positions']:
                          joint_positions[i] = real_time_data['joint_positions'][name]

                  joint_state_msg.position = joint_positions
                  publisher.publish(joint_state_msg)

                  print(f"Joint positions: {joint_positions}")
              
              elif evt.event_type == MCPEventType.RigidBodyUpdated:
                  print('rigid body updated')
              else:
                  print('unknow event')    
    rclpy.spin(node)
  except Exception as e:
          node.get_logger().error(f"Error publishing joint state: {e}")      
  finally:
      app.close()
      node.destroy_node()
      rclpy.shutdown()  

if __name__ == '__main__':
 ros2_joint_state_publisher()