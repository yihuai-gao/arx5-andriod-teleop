import signal
import sys
import rospy
import tf2_ros
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import TransformStamped

sys.path.append("/home/yihuai/robotics/repositories/arx5-sdk/python")
import arx5_interface as arx5
from scipy.spatial.transform import Rotation as R
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

controller = arx5.Arx5CartesianController("L5", "can0", "/home/yihuai/robotics/repositories/arx5-sdk/models/arx5.urdf")
controller.reset_to_home()
@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('message')
def handle_message(data):
    emit('echo', data['timestamp'])
    if 'position' in data:
        position = data['position']
        orientation = data['orientation']
        
        print(f"{position['x']:.2f}, {position['y']:.2f}, {position['z']:.2f}")
        quaternion = np.array([orientation['x'], orientation['y'], orientation['z'], orientation['w']])
        # Create a rotation object from the quaternion
        rotation = R.from_quat(quaternion)
        # Convert to roll, pitch, yaw (in radians)
        roll, pitch, yaw = rotation.as_euler('xyz', degrees=False)
        pose6d = np.array([position['x'], position['y'], position['z'], roll, pitch, yaw])
        home_pose = controller.get_home_pose()
        timestamp = controller.get_timestamp()
        eef_cmd = arx5.EEFState()
        pose6d[:3] /= 3 # To reduce movement
        eef_cmd.pose_6d()[:] = pose6d + home_pose
        eef_cmd.timestamp = timestamp + 0.05
        if max(eef_cmd.pose_6d()) > 1:
            print("out of range")
        else:
            controller.set_eef_cmd(eef_cmd)

        publish_pose(position, orientation)

# Initialize the ROS node
rospy.init_node('pose_publisher', anonymous=True)

# Create a TransformBroadcaster object
br = tf2_ros.TransformBroadcaster()

socketio.run(app, host='0.0.0.0', port=5000)
