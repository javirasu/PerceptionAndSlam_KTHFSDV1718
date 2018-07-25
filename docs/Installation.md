# Installation 

## Ubuntu
Check [this tutorial](https://youtu.be/qNeJvujdB-0) to install Ubuntu 16.04 as dual boot besides Windows 10. Choose the default option (install ubuntu alongside windows)

For installation of Ubuntu on MSI (team laptop), follow [this tutorial](https://medium.com/@gentra/how-to-install-ubuntu-16-04-on-msi-ge62-6qc-ae4f30f50465). You might need to install latest Nvidia drivers. Check [this link](https://tecadmin.net/install-latest-nvidia-drivers-ubuntu/) on how to do that. Check [this link](http://www.nvidia.com/Download/index.aspx?lang=en-us) to find the latest Nvidia drivers.

## Cuda
- Check cuda verison by typing `nvcc -- version` on a terminal. 
- Download [cuda drivers](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=debnetwork
) from Nvidia website and follow their installation instructions. 
- For command 4, 

`sudo apt-get install cuda` 

becomes 

`sudo apt-get install cuda-9-0`

## Tensorflow
We'll use Tensorflow GPU. First, [install dependencies](https://www.tensorflow.org/install/install_linux#tensorflow_gpu_support) for it. Cuda toolkit and Nvidia drivers were already installed in sections above. 

For cuDNN libraries , go to [this link](https://developer.nvidia.com/rdp/cudnn-archive) and select cuDNN version corresponding to cuda 9.0. Download the following:
- cuDNN 7.x Runtime library for ubuntu 16.04 (deb)
- cuDNN 7.x Developer library for ubuntu 16.04 (deb)
- cuDNN 7.x Code Samples and user guide for ubuntu 16.04 (deb)
- (optional) cuDNN developer guide, install guide and release notes.

While installing cuda command line tools:

`sudo apt-get install cuda-command-line-tools`

becomes

`sudo apt install cuda-command-line-tools-9-0`

*Add this path to the LD_LIBRARY_PATH environmental variable*. This basically means that you need to modify the *.bashrc* file (it's a hidden file in home folder of ubuntu). Go to home, press Ctrl + H to reveal hidden files, open .bashrc file and paste the following at the end:

`export LD_LIBRARY_PATH=${LD_LIBRARY_PATH:+${LD_LIBRARY_PATH}:}/usr/local/cuda/extras/CUPTI/lib64`

Now, we can finally install Tensorflow GPU. We recommend using [Native pip](https://www.tensorflow.org/install/install_linux#InstallingNativePip) method. 

Install pip: `sudo apt-get install python-pip`

Install tensorflow (you might want same version as on Embedded computer/Jetson TX2): `pip install tensorflow-gpu == 1.8`

**NOTE:** Throughout course of development, it's highly recommended to stick to one version of python (either 2 or 3). **We recommend installing everything for python 2** as it doesn't cause clashes with ROS down the line.

## Tensorflow Object Detection API
Clone the Tensorflow models repository. It's awesome!

`git clone https://github.com/tensorflow/models` 

Install [all dependencies](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/installation.md
) for object detection api.

**Potential issues**

- There might be a need to update protobuf drivers. [Follow this](https://gist.github.com/sofyanhadia/37787e5ed098c97919b8c593f0ec44d8)
- https://github.com/tensorflow/models/issues/4002
- while adding libraries to python path, change pwm to exact path, eg:

`export PYTHONPATH="${PYTHONPATH}:/home/ajinkya/models/research:/home/ajinkya/models/research/slim/"`

## ROS
[Install ROS](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Robot Operating System) and check this handy [tutorial series](https://www.youtube.com/playlist?list=PLk51HrKSBQ8-jTgD0qgRp1vmQeVSJ5SQC) to learn how to use it!

# Running Instructions
Most of the software we need is installed. Now we'll look at how to run the code.

## Create a Workspace
All our code would reside in what is called a [catkin workspace](http://wiki.ros.org/catkin/workspaces). To create a workspace, follow [this link](http://wiki.ros.org/catkin/Tutorials/create_a_workspace) 

`mkdir -p ~/PerceptionAndSlam_KTHFSDV1718`

`cd ~/PerceptionAndSlam_KTHFSDV1718/`

`catkin_make`

Next

`cd ~/PerceptionAndSlam_KTHFSDV1718/`

`git clone https://github.com/javirrs/PerceptionAndSlam_KTHFSDV1718.git`

This creates a folder named 'PerceptionAndSlam_KTHFSDV1718' inside our catkin workspace (which for now is also called PerceptionAndSlam_KTHFSDV1718). Now rename thie inner folder from PerceptionAndSlam_KTHFSDV1718 to 'src' (source, as this is where our source code will reside). 

## More Installations!
We're not done yet! To run various sensors with our system we need to install ros-wrappers for them. A ros-wrapper is a ros package with pre-written code supplied by manufacturers/open-source heros. This makes our life easier when interfacing with the sensors. We'll see how. Install these wrappers by cloning their repos in the src folder.
- [Zed camera](https://github.com/stereolabs/zed-ros-wrapper)
- [VLP 16 Lidar](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16)
- [Xsens imu and gps](https://github.com/xsens/xsens_mti_ros_node)

There are a few additional wrappers needed. Just look at the error when running catkin_make (we'll get to it soon), google them and clone their github repos in src
- [uuid_msgs](https://github.com/ros-geographic-info/unique_identifier)
- [geographic_msgs](https://github.com/ros-geographic-info/geographic_info)

Finally, 

`cd ~/PerceptionAndSlam_KTHFSDV1718/`

`catkin_make`

## Launching the Program
We're here. Congratulations on reaching this far! To launch the program we use something called a .launch file. You can imagine it as being a file containing launch sequence for a rocket-launch. All the launch files are to be kept in *perc_slam_launch* package. This package contains two folders, *launch* and *rosbags*. A rosbag is like a video file, but instead of a video, it has the ability to record all (or chosen) ros topics. These can then be played back and we can run innumerable experiments on them! 

Now, before ros can recognize all packages in our workspace, we need to source our workspace to .bashrc. 

`cd ~/PerceptionAndSlam_KTHFSDV1718/`

`echo  'source devel/setup.bash' >> ~/.bashrc`

`source ~/.bashrc`

Next time onwards, ros will know the contents of our workspace. Now finally, run

`roslaunch perc_slam_launch play_rosbag.launch`

to play one of the rosbags, or connect all the sensors and run

`roslaunch perc_slam_launch perc_slam.launch`

to run the system on live sensor data.
