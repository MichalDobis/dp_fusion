//ros
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"

#include <iostream>
#include <fstream>
#include <fstream>


//#include <phoxi_camera/TriggerImage.h>
//boost
#include <boost/foreach.hpp>


#include <cmath>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/boost.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <chrono>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/filters/voxel_grid.h>

//general
//#include "PhoXi.h"

using namespace std;

typedef struct Target{
    Eigen::Quaternionf rotation;
    Eigen::Vector3f translation;
} Target;

class CloudMeshing{

public:
    CloudMeshing::CloudMeshing(): t(),
                                  cloud_out_final(new pcl::PointCloud<pcl::PointXYZRGB>){
        ROS_INFO("Cakam na servis pre ziskanie cloudu");
        listen = nh.advertiseService("/parse_cloud", &CloudMeshing::capture_data,this);
        //phoxi_client = nh.serviceClient<phoxi_camera::TriggerImage>("/phoxi_camera/trigger_image");
        k = 1;
    }
    CloudMeshing::~CloudMeshing(){
    }

private:

    int k;
    tf::TransformListener t;

    ros::NodeHandle nh;
    ros::ServiceServer listen;
    ros::ServiceClient phoxi_client;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out_final;

    uint16_t calcMedianI(vector<uint16_t> data);
    bool capture_data(std_srvs::Trigger::Request  &req,
                      std_srvs::Trigger::Response &res);
    void CloudMeshing::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,
                                           pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud_out,
                                           Target target);
};

void CloudMeshing::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr & pointcloud_out,
                                       Target target) {



    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    Eigen::Matrix3f rotacia(target.rotation.toRotationMatrix());
    std::cout << rotacia << std::endl;

       for (int i = 0; i<3;i++){
        for (int x = 0; x<3;x++){
           transformation(i,x) = rotacia(i,x);
        }
    }

    transformation(0,3) = target.translation.x()*1000;
    transformation(1,3) = target.translation.y()*1000;
    transformation(2,3) = target.translation.z()*1000;

    std::cout << transformation.matrix() << std::endl;

    pcl::transformPointCloud (*pointcloud, *pointcloud_out, transformation);

    return;
}


bool CloudMeshing::capture_data(std_srvs::Trigger::Request  &req,
                               std_srvs::Trigger::Response &res) {



    while(ros::ok()) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr1(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGB>);

        if (pcl::io::loadPLYFile<pcl::PointXYZRGB>(
                "/home/controller/catkin_ws/clouds/cloud_from_phoxi/cloud" + to_string(k) + ".ply",
                *point_cloud_ptr1) == -1) //* load the file
        {
            PCL_ERROR ("Couldn't read file I! \n");
            return (-1);
        } else {
            Target my_target1;

            std::ifstream input_file1;
            std::ifstream input_file2;

            input_file1.open("/home/controller/catkin_ws/clouds/point_for_phoxi/cloud" + to_string(k) + ".txt");
            if (!input_file1){
                ROS_ERROR("ERROR OPENING FILE %s","/home/controller/catkin_ws/clouds/point_for_phoxi/cloud" + to_string(k) + ".txt");
                return false;
            }
            //    input_file2.open("/home/controller/catkin_ws/clouds/cloud_from_phoxi/cloud"+to_string(k)+".txt");

            input_file1 >> my_target1.translation.x() >> my_target1.translation.y() >> my_target1.translation.z();
            input_file1 >> my_target1.rotation.x() >> my_target1.rotation.y() >> my_target1.rotation.z()
                        >> my_target1.rotation.w();

            this->transformPointCloud(point_cloud_ptr1, cloud_out, my_target1);
            std::cout << cloud_out->points.size();
            for (int i = 0; i < cloud_out->points.size(); i++) {

                if (cloud_out->points[i].z == cloud_out->points[i].z) {
                    cloud_out_final->points.push_back(cloud_out->points[i]);
                }

            }

            ROS_INFO("Data sucessfully loaded");
            if (k == 11) {
                ROS_INFO("Generating Ended");
                pcl::io::savePLYFileASCII("/home/controller/catkin_ws/clouds/final_output.ply", *this->cloud_out_final);
                break;
            }
            //pcl::io::savePLYFileASCII ("/home/controller/catkin_ws/clouds/output"+to_string(k)+".ply", *this->cloud_out_final);
            input_file1.close();
            input_file2.close();
            k++;
        }
    }
    return true;


//    Target my_target;
//
//    this->transformPointCloud(point_cloud_ptr1, my_target);



    /*tf::StampedTransform kinect_transform;

    std::string baseLinkFrame = "/base_link";
    std::string kinFrame = "/tool0";

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
            if (CloudMeshing::t.waitForTransform(kinFrame, now, kinFrame, now, baseLinkFrame, ros::Duration(1))) {
                //msg->header.stamp.setNow(now);
                break;
            }
        }
        // baselink v kamerovych suradniciach
        try {
            CloudMeshing::t.lookupTransform(baseLinkFrame, kinFrame,
                                           ros::Time(0), kinect_transform);

        }
        catch (tf::TransformException ex) {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        std::ofstream output_file("/home/controller/catkin_ws/calibracia_tomas/point_" + to_string(k) + ".txt",std::ofstream::out);

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
    }*/
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cloud_meshing");
    CloudMeshing cloudmeshihng;
    ros::spin();
}
