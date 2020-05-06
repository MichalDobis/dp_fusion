//
// Created by controller on 8/2/17.
//

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Trigger.h>
#include <phoxi_camera/GetFrame.h>
#include <geometry_msgs/Pose.h>


#include <ceres/problem.h>
#include <ceres/cost_function.h>

//
#include <ceres/autodiff_cost_function.h>
#include <ceres/local_parameterization.h>
#include <ceres/solver.h>
#include <Eigen/Geometry>
#include "pho_robot_loader/constants.h"

using namespace std;

class CalibrationDataLogger{

public:

    CalibrationDataLogger():nh("~"),
    output_file("/home/controller/catkin_ws/calibration_data.txt"){


        calibration_data_server = nh.advertiseService("generate_calib_file", &CalibrationDataLogger::capture_data,this);
        phoxi_camera_get_frame_client = nh.serviceClient<phoxi_camera::GetFrame>("/phoxi_camera/get_frame");

        if(!phoxi_camera_get_frame_client.exists()){
            ROS_ERROR_STREAM("PLEASE STArt phoxi camera node and connect camera");
            ros::shutdown();
        }
        ROS_INFO("CALIBRATION LOGGER CREATED");
    }
    ~CalibrationDataLogger(){
        output_file.close();
    }



private:


    ros::NodeHandle nh;

    ros::ServiceServer calibration_data_server;
    ros::ServiceClient phoxi_camera_get_frame_client;

    ofstream output_file;


    bool capture_data(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool getCameraInMarkerSpacePos(Eigen::Vector3d & ObservedCameraTranslations, Eigen::Quaterniond & ObservedCameraQuaternions);


};

bool CalibrationDataLogger::getCameraInMarkerSpacePos(Eigen::Vector3d & ObservedCameraTranslations, Eigen::Quaterniond & ObservedCameraQuaternions){


    phoxi_camera::GetFrame frame_data;
    frame_data.request.in = -1;

    //logging markerspace_position
    if (this->phoxi_camera_get_frame_client.call(frame_data)) {

        Eigen::Matrix3d rot_matrix;

        ObservedCameraTranslations.x() = frame_data.response.translation_vec[0]/1000;
        ObservedCameraTranslations.y() = frame_data.response.translation_vec[1]/1000;
        ObservedCameraTranslations.z() = frame_data.response.translation_vec[2]/1000;

        for(int x =0;x<3;x++) {
            for (int y = 0; y < 3; y++) {
                rot_matrix(x, y) = frame_data.response.rotation_matrix.matrix[x].row[y];
            }
        }

        ObservedCameraQuaternions =  Eigen::Quaterniond(rot_matrix);

        return true;

    } else {
        ROS_ERROR_STREAM("Wrong data read from phoxi camera node");
        return false;
    }



}


bool CalibrationDataLogger::capture_data(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {

    //logging tool point

    ros::Duration timeout(5);

    boost::shared_ptr<const geometry_msgs::Pose> robotPoint = ros::topic::waitForMessage<geometry_msgs::Pose>(
            pho_robot_loader::PHOTONEO_TOPICS::TOOL_POSE, ros::Duration(2));
    if (!robotPoint) {
        ROS_ERROR("Robot tool pose timeout");
        return false;
    } else {


        //logging markerspace_position
        Eigen::Vector3d translation;
        Eigen::Quaterniond rotation;

        if (getCameraInMarkerSpacePos(translation, rotation)) {

            this->output_file << robotPoint->position.x << " " << robotPoint->position.y << " "
                              << robotPoint->position.z << " " << robotPoint->orientation.x << " "
                              << robotPoint->orientation.y << " " << robotPoint->orientation.z << " "
                              << robotPoint->orientation.w << " ";


            this->output_file << translation.x() << " " << translation.y() << " "
                              << translation.z() << " " << rotation.x() << " "
                              << rotation.y() << " " << rotation.z() << " "
                              << rotation.w() << "\n";
            return true;
        }
    }

    return false;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibration_data_logger");

    ros::NodeHandle node("~");

    CalibrationDataLogger calibration_data_logger;

    ros::spin();
    return 0;
}
