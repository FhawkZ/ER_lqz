import json
from mocap_robotapi import *

import rospy
from sensor_msgs.msg import JointState

def ros1_joint_state_publisher():
  # init ros node
  rospy.init_node("real_time_joint_state_publisher", anonymous=True)
  publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
  rate = rospy.Rate(10)  # set fps to 10Hz
  # Joints name list
  json_file_path = './retarget.json'
  with open(json_file_path, 'r') as file:
      data = json.load(file)
  joint_names = [item['retargetJoint'] for item in data['retargetJoints']]
  
  robot = MCPRobot(open(json_file_path).read())
  app = MCPApplication()
  settings = MCPSettings()
  settings.set_udp(7012)
  settings.set_bvh_rotation(0)
  app.set_settings(settings)
  app.open()
  try:
    while not rospy.is_shutdown():
        evts = app.poll_next_event()
        for evt in evts:
            if evt.event_type == MCPEventType.AvatarUpdated:
                avatar = MCPAvatar(evt.event_data.avatar_handle)
                robot.update_robot(avatar)
                robot.run_robot_step()
                #print (robot.get_robot_ros_frame_json())
                
                # get realtime data
                real_time_data = json.loads(robot.get_robot_ros_frame_json()[0])
                #joint_positions = real_time_data['joint_positions']

                # create and publish JointState
                joint_state_msg = JointState()
                joint_state_msg.header.stamp = rospy.Time.now()
                joint_state_msg.name = joint_names

                # init joint positions
                joint_positions = [0.0] * len(joint_names)

                # fill the position
                for i, name in enumerate(joint_names):                    
                    if name in real_time_data['joint_positions']:
                        joint_positions[i] = real_time_data['joint_positions'][name]

                joint_state_msg.position = joint_positions
                # publish
                publisher.publish(joint_state_msg)
                rospy.loginfo(f"Published joint positions: {joint_positions}")
            
            elif evt.event_type == MCPEventType.RigidBodyUpdated:
                print('rigid body updated')
            else:
                print('unknow event')
        rate.sleep()
  except rospy.ROSInterruptException:
      pass      
  app.close()
  
if __name__ == '__main__':
  ros1_joint_state_publisher()