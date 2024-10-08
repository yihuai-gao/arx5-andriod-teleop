# iPhone teleop code snippet for Yihuai

This code will be cleaned up and open sourced soon, please do not share.

## Setup

```bash
mamba create -n ros-noetic
mamba activate ros-noetic
conda config --env --add channels conda-forge
conda config --env --add channels robostack-staging
conda config --env --remove channels defaults
mamba install ros-noetic-desktop
pip install flask flask_socketio
```

## Usage

Start ROS:

```bash
mamba activate ros-noetic
roscore
```

Start Flask server:

```bash
mamba activate ros-noetic
python app_rviz.py
```

Start RViz:

```bash
mamba activate ros-noetic
rviz
```

Then follow these steps:
1. Open the "XR Browser" app on iPhone and navigate to `http://your-server-hostname:5000`.
2. Go to RViz: Add > TF
3. Back on iPhone, press "Start AR"
4. Tap on the screen to send the current iPhone pose to the server, which will be visualized in RViz
