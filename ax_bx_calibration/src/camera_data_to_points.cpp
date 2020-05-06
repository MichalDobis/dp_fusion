//
// Created by controller on 8/17/17.
//

#include "ros/ros.h"

//STD
#include <iostream>
#include <fstream>
#include <fstream>

#include <ceres/problem.h>
#include <ceres/cost_function.h>
#include "ceres_extensions.h"
//
#include <ceres/autodiff_cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/solver.h>


#define OBSERVATIONS 6

int main(int argc,char** argv){

    ros::init(argc, argv, "calibration_node");

    std::ifstream input_file;
    input_file.open("/home/controller/catkin_ws/src/pho_dp_fusion/ax_bx_calibration/inputs_examples/data.txt",std::ifstream::in);
    if (input_file){
        ROS_INFO_STREAM("INPUT FILE OPENED");
    }
    else{

        ROS_ERROR_STREAM("problem opening file");
        ros::shutdown();
    }
    std::ofstream outputfile;
    outputfile.open("/home/controller/catkin_ws/src/pho_dp_fusion/ax_bx_calibration/outputs/output_data.txt",std::ofstream::out);
    if(outputfile){
        ROS_INFO_STREAM("OUTPUT FILE OPENED");
    }
    else{

        ROS_ERROR_STREAM("problem opening file");
        ros::shutdown();
    }
    Eigen::Vector3d ObservedCameraTranslations;

    for(int i=0;i<OBSERVATIONS;i++){



        input_file >> ObservedCameraTranslations.x();
        input_file >> ObservedCameraTranslations.y();
        input_file >> ObservedCameraTranslations.z();


        Eigen::Matrix3d rot_matrix;

        for(int x =0;x<3;x++) {
            for (int y = 0; y < 3; y++) {
                input_file >> rot_matrix(x, y);
            }
        }

        Eigen::Quaterniond ObservedCameraQuaternions(rot_matrix);

        ROS_INFO_STREAM("OBSERVATION:");
        ROS_INFO_STREAM(ObservedCameraTranslations);
        ROS_INFO_STREAM(rot_matrix);

        outputfile << ObservedCameraTranslations.x()/1000 << " " << ObservedCameraTranslations.y()/1000 << " " << ObservedCameraTranslations.z()/1000;
        outputfile << " " << ObservedCameraQuaternions.x() << " " << ObservedCameraQuaternions.y() << " " << ObservedCameraQuaternions.z() << " " << ObservedCameraQuaternions.w() << std::endl;
    }
    input_file.close();
    outputfile.close();

    ROS_INFO_STREAM("PRESS ENTER TO EXIT PROGRAM");
    getchar();

}