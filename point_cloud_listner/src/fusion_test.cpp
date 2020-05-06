//ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <iostream>
#include <fstream>
#include <fstream>


#include <phoxi_camera/TriggerImage.h>

//boost
#include <boost/foreach.hpp>

//general
#include "PhoXi.h"

using namespace std;

class ScannerComm{

public:
    ScannerComm::ScannerComm(): t(){
        ROS_INFO("Cakam na servis pre ziskanie cloudu");
        listen = nh.advertiseService("/scan", &ScannerComm::capture_data,this);
        phoxi_client = nh.serviceClient<phoxi_camera::TriggerImage>("/phoxi_camera/trigger_image");
        k = 1;
    }
    ScannerComm::~ScannerComm(){
    }

private:

    int k;
    tf::TransformListener t;

    ros::NodeHandle nh;
    ros::ServiceServer listen;
    ros::ServiceClient phoxi_client;

    uint16_t calcMedianI(vector<uint16_t> data);
    bool capture_data(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);

};



bool ScannerComm::capture_data(std_srvs::Trigger::Request  &req,
                              std_srvs::Trigger::Response &res) {

    tf::StampedTransform kinect_transform;

    std::string baseLinkFrame = "/base_link";
    std::string kinFrame = "/kinect_frame";

	phoxi_camera::TriggerImage srv;

    if (this->phoxi_client.call(srv))
    {
        ROS_INFO("Service Sucesfully called");
    }
    else
    {
        ROS_ERROR("Problem calling service");
        return 1;
    }

    if(srv.response.success) {
        ROS_ERROR("Scan complete listening transformation");
        ros::Time now(0);

        while (nh.ok()) {
            std::string *error_code;
            //ROS_INFO_STREAM(now);
            if (ScannerComm::t.waitForTransform(kinFrame, now, kinFrame, now, baseLinkFrame, ros::Duration(1))) {
                //msg->header.stamp.setNow(now);
                break;
            }
        }
        // baselink v kamerovych suradniciach
        try {
            ScannerComm::t.lookupTransform(baseLinkFrame, kinFrame,
                                          ros::Time(0), kinect_transform);

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        std::ofstream output_file("/home/controller/catkin_ws/clouds/point_for_phoxi/cloud" + to_string(k) + ".txt",std::ofstream::out);

        //output_file << "Translation(m)" << "\n";
        output_file << kinect_transform.getOrigin().x() << " " << kinect_transform.getOrigin().y() << " "
                    << kinect_transform.getOrigin().z() << "\n";
        //output_file<< "Rotation(rad)" << "\n";
        output_file << kinect_transform.getRotation().x() << " " << kinect_transform.getRotation().y() << " "
                    << kinect_transform.getRotation().z() << " "
                    << kinect_transform.getRotation().w() << "\n";

        //std::cout << RobotRQuaternion.coeffs() << "\n";
        //result rotation = RobotRMat (as Eigen::Matrix3D), rotation = RobotRQuaternion (as Eigen::Quaterniond) , translation = RobotTranslationVec (as Eigen::Vector3D), translation = TrackerTranslationVector (as double[3])

        output_file.close();


        ROS_INFO_STREAM(kinect_transform.getOrigin().x() << " " << kinect_transform.getOrigin().y() << " "
                                                         << kinect_transform.getOrigin().z());
        ROS_INFO_STREAM(kinect_transform.getRotation().x() << " " << kinect_transform.getRotation().y() << " "
                                                           << kinect_transform.getRotation().z() << " "
                                                           << kinect_transform.getRotation().w());
        k++;
        res.success = true;
        return true;
    }
    else{
        ROS_ERROR("Problem  with scanning");
        res.success = false;
        return false;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ScannerComm scannercomm;
    ros::spin();
}
