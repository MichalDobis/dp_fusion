# Basic info
AX BX calibration node used for robot camera calibration.
# Inputs
As input you can use file of points or transformation matrixes in format Eigen::Matrix4D. See input examples. 
In every line there is set of points 
first is: tool0 -> base 
second is: position of pattern -> camera_space
Every point is defined by his position (XYZ) and orientation as quaternion(XYZW)

You can also specify Eigen matrixes as input. You define matrixes 4x4  first is matrix A and second is matrix B.

# Prerequirements

For succesfull package compilation you need to have [ceres_1.12.0](https://github.com/photoneo/3rdParty_Generator/releases/download/u16/ceres-solver-1.12.0.tar.bz2)
in your 3rdparty directory

You also need to install 

`sudo apt-get install libgoogle-glog-dev`


# Launch

After successful build you can launch your calibration node by calling. 

`roslaunch ax_bx_calibration ax_bx_calibration.launch`

WARNING: You need to specify paths for calibration file output and input in your launchfile. 
At first check parameters in this file and specify whether you want to use points or matrixes for your calibration.

