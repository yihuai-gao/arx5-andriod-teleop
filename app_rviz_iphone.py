import signal
import sys
import rospy
import tf2_ros
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import TransformStamped


# sys.path.append("your/path/to/arx5-sdk/python")
sys.path.append("/home/yihuai/Robotics/Repositories/arx5-sdk/python")

import arx5_interface as arx5
from transforms3d import quaternions, affines,euler
import numpy as np
def signal_handler(sig, frame):
    sys.exit()

# Set the signal handler for SIGINT
signal.signal(signal.SIGINT, signal_handler)

def publish_pose(position, orientation):
    # Create a TransformStamped message
    t = TransformStamped()

    # Fill the header
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = 'map'
    t.child_frame_id = 'viewer'

    # Fill the position
    t.transform.translation.x = position['x']
    t.transform.translation.y = position['y']
    t.transform.translation.z = position['z']

    # Fill the orientation
    t.transform.rotation.x = orientation['x']
    t.transform.rotation.y = orientation['y']
    t.transform.rotation.z = orientation['z']
    t.transform.rotation.w = orientation['w']

    # Broadcast the transform
    br.sendTransform(t)

app = Flask(__name__)
socketio = SocketIO(app)

# controller = arx5.Arx5CartesianController("X5 or L5", "your can interface", "your/path/to/arx5-sdk/models/arx5.urdf")
controller = arx5.Arx5CartesianController("L5", "can3", "/home/yihuai/Robotics/Repositories/arx5-sdk/models/arx5.urdf")

controller.reset_to_home()

WINDOW_SIZE = 5
prev_eef_commands = np.zeros([WINDOW_SIZE, 6])    


@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('message')
def handle_message(data):
    emit('echo', data['timestamp'])
    if 'position' in data:
        position = data['position']
        orientation = data['orientation']
        
        phone_pose = affines.compose(
            T=[position['x'], position['y'], position['z']],
            R=quaternions.quat2mat([orientation['w'], orientation['x'], orientation['y'], orientation['z']]),
            Z = [1, 1, 1]
        )
        
        transform_mat = np.array([
            [0, 0, 1, 0], 
            [1, 0, 0, 0], 
            [0, 1, 0, 0],
            [0, 0, 0, 1]]
            )
        

        transformed_phone_mat = np.dot(transform_mat, phone_pose)
        position["x"] = transformed_phone_mat[0, 3]
        position["y"] = transformed_phone_mat[1, 3]
        position["z"] = transformed_phone_mat[2, 3]
        transformed_quat = quaternions.mat2quat(transformed_phone_mat[:3, :3])
        orientation["w"] = transformed_quat[0]
        orientation["x"] = transformed_quat[1]
        orientation["y"] = transformed_quat[2]
        orientation["z"] = transformed_quat[3]

        r, p, y = euler.quat2euler([orientation['w'], orientation['x'], orientation['y'], orientation['z']])
        r -= np.pi/2
        y -= np.pi/2
        r, p = p, r
        r = -r
        pose_euler = np.array([r, p, y])
        r, p, y = np.mod(pose_euler + np.pi, 2*np.pi) - np.pi

        pose6d = np.array([position["x"], position["y"], position["z"], r, p, y])
        global prev_eef_commands
        prev_eef_commands = np.vstack([prev_eef_commands[1:], pose6d])
        avg_eef_command = np.mean(prev_eef_commands, axis=0)


        home_pose = controller.get_home_pose()
        timestamp = controller.get_timestamp()
        eef_cmd = arx5.EEFState()
        eef_cmd.pose_6d()[:] = avg_eef_command + home_pose
        eef_cmd.timestamp = timestamp + 0.1 # delay a little bit to improve smootheness
        print(eef_cmd.pose_6d())
        controller.set_eef_cmd(eef_cmd) 
            
        publish_pose(position, orientation)

# Initialize the ROS node
rospy.init_node('pose_publisher', anonymous=True)

# Create a TransformBroadcaster object
br = tf2_ros.TransformBroadcaster()

np.set_printoptions(precision=3, suppress=True)

socketio.run(app, host='0.0.0.0', port=5000)
