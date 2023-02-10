# This is a test project to read camera data from Gen3 arm.

These examples are meant for internal use, but feel free to use them to help clients.
Some of the projects here use ROS while others use kortex.
This has been tested on Ubuntu 20.04


## Setup
In your ros kortex catkin workspace install the repo:
```sh
cd ~/kortex_ws/src
git clone https://github.com/IlianaKinova/fiducials.git
python3 -m venv ./fiducials/.venv
source ./fiducials/.venv/bin/activate
sudo apt-get install gstreamer1.0*
pip install pip --upgrade
pip install -r fiducials/requirements.txt
cd ../
```

Then build the workspace:
```sh
catkin_make
```

And finally, source the workspace:
```sh
source devel/setup.bash
```

## Where to go from now?
There are a few code examples here. Some of which do not work.

### createFiducials.py
This is a simple script that outputs a pdf displaying fiducials.

### fiducialsDetect.py
This script will track the fiducials made in the previous script.

### helloworld.py
This was a code example I used to test if gstreamer was able to open the streams.

### simpleCap.py
This will open your webcam and display the image, you can change the source in the parameters of `VideoCapture`

### simpleDepthRosCap.py
This example uses ROS to capture the depth stream. You first need to start the kinova_vision_rgbd.launch.