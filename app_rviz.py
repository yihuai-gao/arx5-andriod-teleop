import signal
import sys
import rospy
import tf2_ros
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
from geometry_msgs.msg import TransformStamped

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

@app.route('/')
def index():
    return render_template('index.html')

@socketio.on('message')
def handle_message(data):
    emit('echo', data['timestamp'])
    if 'position' in data:
        position = data['position']
        orientation = data['orientation']
        publish_pose(position, orientation)

# Initialize the ROS node
rospy.init_node('pose_publisher', anonymous=True)

# Create a TransformBroadcaster object
br = tf2_ros.TransformBroadcaster()

socketio.run(app, host='0.0.0.0', port=5000)
