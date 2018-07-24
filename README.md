# PerceptionAndSlam_KTHFSDV1718
## Background
Welcome to the perception and slam system of the KTH Formula Student Driverless project for the year 2017-18! So, what do we mean by *Perception* and *Slam*? Imagine a self-driving race car standing on an endurance track of FSD. The first thing car needs to do is to ‘perceive’ traffic cone landmarks. We use zed stereo camera as our perception sensor. After the cones have been detected, the car needs to plot them on a map and simultaneously needs to find its global position in the map, which is called SLAM (Simultaneous Localization & Mapping). The map information can be used by navigation team for path-planning and actuation. 

When the first ever KTH FSD team assembled in Sep 2017, we were working with a clean slate. We had no idea where to even start. We hope that this repository would serve as a solid base for next batch of FSD enthusiasts!

## Installation
Refer to [Installation](docs/Installation.md) for a detailed list of dependencies and instructions

## Description 
This section outlines the flow of logic in the system. For a detailed description of every node, follow the links below:
-	[Perception](docs/perception.md)
-	[Localization](docs/localization.md)
-	[Mapping](mapping.md)
-	Visualization

<p align="center">
  <img src="docs/perc_slam_architecture1.png" width=879 height=393>
</p>

In the figure shown above, every blue box is a _ros package_. Every ros package contains a _ros node_, henceforth referred to as a _node_ or _block_. The text above every arrow is a _ros topic_. To start off, we have the following array of sensors:
-	A zed stereo camera. It publishes colored and depth images (a depth image is a 2D image with value of every pixel equal to the depth calculated by disparity of stereo camera).
-	A velodyne lidar (VLP-16). It publishes numpy array of 3D coordinates returned by its 16 laser beams.
-	A xsens IMU (inertial measurement unit). It provides odometry in form of linear acceleration, angular velocities and orientation.
-	A Xsens GPS embedded in IMU. It provides a pose estimate.

In our current version, we utilize the stereo camera and imu. 

As we saw in the self-driving car example, the first step is always perception. The **perception_pipeline** node subscribes to image and depth information published by the zed camera. The node passes every image through a convolutional neural network with SSD-mobile net architecture. The CNN is trained to detect traffic cones in an image and promptly returns a list of bounding boxes where they’re present. Thereafter the node infers the color and calculates the depth of every traffic cone candidate. It publishes an array of detected cones in camera frame.

The **robot_localization** node is a package provided by ros. It fuses the odometry given by zed camera, the imu (and possibly wheel encoders) and publishes a filtered odometry. This odometry is used by the global_mapping node to plot the global position of the car.

The **reactive_mapping** node takes detected cones as input. It performs first level of filtering/outlier rejection and publishes a local map (or a reactive map, i.e. a map where the car is always at the origin).

The **global_mapping** node takes reactive map as input and performs second level of filtering/outlier rejection. It publishes a global map (i.e. a map containing all the cones, the car and with origin being the position of center of mass of the car at t = 0).

The **rviz_visualizer** node takes the output of all the nodes and plots them in rviz, a handy visualization tool provided by ros.

## Citation


    @Misc{gpy2014,
      author =   {{Rasines, Javier and Khoche, Ajinkya}},
      title =    {Perception and SLAM for FormulaStudent 17/18 competition},
      howpublished = {\url{https://github.com/javirrs/PerceptionAndSlam_KTHFSDV1718}},
      year = {since 2012}
    }
    
## License
[GNU AGPL v3.0](LICENSE)
