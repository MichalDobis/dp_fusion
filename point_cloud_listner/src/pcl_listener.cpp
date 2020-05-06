//ros
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <tf/transform_listener.h>
#include "tf_conversions/tf_eigen.h"
#include <sensor_msgs/PointCloud2.h>
#include "pcl_ros/transforms.h"
#include "pcl_ros/impl/transforms.hpp"

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include "pcl/io/file_io.h"
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>

//boost
#include <boost/foreach.hpp>

//general
//#include "PhoXi.h"

using namespace std;

class ParseCloud{

public:
    ParseCloud::ParseCloud() : t(){
        ROS_INFO("Cakam na servis pre ziskanie cloudu");
        listen = nh.advertiseService("/capture_data", &ParseCloud::capture_data,this);
        k = 0;
    }
    ParseCloud::~ParseCloud(){
    }

private:

    int k;
    tf::TransformListener t;

    ros::NodeHandle nh;
    ros::ServiceServer listen;

    uint16_t calcMedianI(vector<uint16_t> data);
    float calcMedianF(vector<float> data);
    bool capture_data(std_srvs::Empty::Request  &req,
                                  std_srvs::Empty::Response &res);
    void transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB>
                                         & pointcloud,const tf::StampedTransform orig_frame);

};

uint16_t ParseCloud::calcMedianI(vector<uint16_t > data)
{
    double median;
    size_t size = data.size();

    sort(data.begin(), data.end());

    if (size  % 2 == 0)
    {
        median = (data[size / 2 - 1] + data[size / 2]) / 2;
    }
    else
    {
        median = data[size / 2];
    }

    return median;
}

float ParseCloud::calcMedianF(vector<float> data)
{
    double median;
    size_t size = data.size();

    sort(data.begin(), data.end());

    if (size  % 2 == 0)
    {
        median = (data[size / 2 - 1] + data[size / 2]) / 2;
    }
    else
    {
        median = data[size / 2];
    }

    return median;
}

void ParseCloud::transformPointCloud(pcl::PointCloud<pcl::PointXYZRGB> & pointcloud,const tf::StampedTransform orig_frame) {


    tf::Quaternion rotation_tf = orig_frame.getRotation().inverse();
    tf::Vector3 translation = orig_frame.getOrigin();
    Eigen::Quaterniond rotation_eigen;
    tf::quaternionTFToEigen(rotation_tf,rotation_eigen);
    Eigen::Matrix3d rot_matrix = rotation_eigen.toRotationMatrix();
    Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();

    for(int i =0;i<3;i++){
        for(int y =0;y<3;y++){
            transform_1(i,y) = rot_matrix(i,y);
        }
    }
    transform_1(0,3) = -translation.getX();
    ROS_INFO_STREAM("POSUN X" << -translation.getX());
    transform_1(1,3) = -translation.getY();
    transform_1(2,3) = -translation.getZ();
    ROS_INFO_STREAM("POSUN Z" << -translation.getZ());
    ROS_INFO_STREAM("Rotacia" << endl << rot_matrix);
    ROS_INFO_STREAM("Rotacia a posun" << endl << transform_1);

    pcl::PointCloud<pcl::PointXYZRGB> transformed_cloud;
    // You can either apply transform_1 or transform_2; they are the same
    //pcl::transformPointCloud(*transformed_cloud , *transformed_cloud, transform_1);
    pcl::transformPointCloud(pointcloud,transformed_cloud, transform_1);
    pointcloud=transformed_cloud;
}

bool ParseCloud::capture_data(std_srvs::Empty::Request  &req,
                              std_srvs::Empty::Response &res) {

    tf::StampedTransform kinect_transform;
    std::ifstream input_file;
    input_file.open("/home/mirec/catkin_ws/calibracia_tomas/point_"+to_string(k)+".txt");

    std::string baseLinkFrame = "/base_link";
    std::string kinFrame = "/kinect_frame";

    sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/phoxi_camera/pointcloud");
    if (msg){
        std::cout<<"Msg received!"<<std::endl;

        sensor_msgs::PointCloud2 cloud_out;
        pcl::PointCloud<pcl::PointXYZRGB> cloud;
        pcl::PointCloud<pcl::PointXYZRGB> new_cloud;
        ros::Time now(0);

        while (nh.ok()) {
            std::string *error_code;
            //ROS_INFO_STREAM(now);
            if (ParseCloud::t.waitForTransform(kinFrame, now, kinFrame, now, baseLinkFrame, ros::Duration(1))) {
                //msg->header.stamp.setNow(now);
                break;
            }
        }
        // baselink v kamerovych suradniciach
        try{
            ParseCloud::t.lookupTransform(baseLinkFrame,kinFrame,
                              ros::Time(0), kinect_transform);

        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            return false;
        }

        ROS_INFO_STREAM(kinect_transform.getOrigin().x() << " " << kinect_transform.getOrigin().y() << " " << kinect_transform.getOrigin().z());
        ROS_INFO_STREAM(kinect_transform.getRotation().x() << " " << kinect_transform.getRotation().y() << " " << kinect_transform.getRotation().z() << " " << kinect_transform.getRotation().w() );
        pcl_ros::transformPointCloud("base_link",kinect_transform,*msg,cloud_out);
        //pcl_ros::transformPointCloud("base_link",*msg,cloud_out,*tf_listener);
        pcl::fromROSMsg (cloud_out, cloud);
        ROS_INFO("Listening");
        for (int i = 0; i<cloud.points.size();i++){

            if (cloud.points[i].z == cloud.points[i].z && cloud.points[i].z < 4){
                new_cloud.points.push_back(cloud.points[i]);
            }

        }
        ROS_INFO_STREAM(new_cloud.points.size());
        //ParseCloud::transformPointCloud(new_cloud,kinect_frame);
        //tf2::doTransform (cloud_in, cloud_out, transform);

        pcl::io::savePLYFileBinary("/home/controller/catkin_ws/clouds/cloud_"+to_string(k)+".ply",new_cloud);
        //printf ("\t(%d)\n", (int)cloud.points.size());
        k++;

    }
    else{
        std::cout<<"No message!"<<std::endl;
        return false;
    }
    input_file.close();
    ROS_INFO_STREAM("Cloud obtained and transformed");
    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ParseCloud parsecloud;
    ros::spin();
}
