# Installation 

## Ubuntu
Check [this tutorial](https://youtu.be/qNeJvujdB-0) to install Ubuntu 16.04 as dual boot besides Windows 10. Choose the default option (install ubuntu alongside windows)

For installation of Ubuntu on MSI (team laptop), follow [this tutorial](https://medium.com/@gentra/how-to-install-ubuntu-16-04-on-msi-ge62-6qc-ae4f30f50465). You might need to install latest Nvidia drivers. Check [this link](https://tecadmin.net/install-latest-nvidia-drivers-ubuntu/) on how to do that. Check [this link](http://www.nvidia.com/Download/index.aspx?lang=en-us) to find the latest Nvidia drivers.

## Cuda
- Check cuda verison by typing `nvcc -- version` on a terminal. 
- Download [cuda drivers](https://developer.nvidia.com/cuda-90-download-archive?target_os=Linux&target_arch=x86_64&target_distro=Ubuntu&target_version=1604&target_type=debnetwork
) from Nvidia website and follow their installation instructions. 
- For command 4, `sudo apt-get install cuda` becomes `sudo apt-get install cuda-9-0`

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

Install tensorflow (same version as is on Jetson TX2): `pip install tensorflow-gpu == 1.6`

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

