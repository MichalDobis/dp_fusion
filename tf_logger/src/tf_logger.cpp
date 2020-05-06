//
// Created by controller on 8/2/17.
//

#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include "ros/ros.h"
#include <tf/transform_listener.h>

using namespace std;


ostream& operator<< (ostream& os, const tf::Quaternion& quat)
{
    os << quat.x() << " " << quat.y() << " " << quat.z() << " " << quat.w();
    return os;
}

ostream& operator<< (ostream& os, const tf::Vector3& trans)
{
    os << trans.x() << " " << trans.y() << " " << trans.z();
    return os;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "tf_logger");

    ros::NodeHandle node("~");

    string baseLinkFrame;
    node.param<string>("baseLinkFrame", baseLinkFrame, "/base_link");
    string odomFrame;
    node.param<string>("odomFrame", odomFrame, "/odom");
    string kinectFrame;
    node.param<string>("kinectFrame", kinectFrame, "/openni_rgb_optical_frame");
    string worldFrame;
    node.param<string>("worldFrame", worldFrame, "/world");
    string outputFileName;
    string path("/home/controller/catkin_ws/");
    node.param<string>("outputFileName", outputFileName, "CalibrationTransformation.txt");
    outputFileName =  path + outputFileName;
    cout << baseLinkFrame << " " << odomFrame << " " << kinectFrame << " " <<
         worldFrame << " " << outputFileName << endl;

    tf::TransformListener t;
    tf::StampedTransform tr_o, tr_i;

    ROS_INFO_STREAM("waiting for initial transforms");
    while (node.ok())
    {
        ros::Time now(ros::Time::now());
        //ROS_INFO_STREAM(now);
        if (t.waitForTransform(baseLinkFrame, now, baseLinkFrame, now, odomFrame, ros::Duration(0.1)))
            break;
        //ROS_INFO("wait");
        //ros::Duration(0.1).sleep();
    }
    ROS_INFO_STREAM("got first odom to baseLink");
    while (node.ok())
    {
        ros::Time now(ros::Time::now());
        //ROS_INFO_STREAM(now);
        if (t.waitForTransform(kinectFrame, now, kinectFrame, now, worldFrame, ros::Duration(0.1)))
            break;
        //ROS_INFO("wait");
        //ros::Duration(0.1).sleep();
    }

    ROS_INFO_STREAM("got first world to kinect");
    sleep(3);

    ros::Rate rate(0.5);
    ofstream ofs(outputFileName.c_str());
    while (node.ok())
    {
        // sleep
        ROS_INFO_STREAM("CITAM a cakam na enter");
        getchar();
        ros::spinOnce();
        rate.sleep();

        // get parameters from transforms
        ros::Time curTime(ros::Time::now());
        ros::Time lastTime = curTime - ros::Duration(2);
        //ROS_INFO_STREAM("curTime: " << curTime << ", lastTime: " << lastTime);

        if (!t.waitForTransform(baseLinkFrame, curTime, baseLinkFrame, lastTime, odomFrame, ros::Duration(3)))
            break;
        if (!t.waitForTransform(kinectFrame, curTime, kinectFrame, lastTime, worldFrame, ros::Duration(3)))
            break;
        try{

            t.lookupTransform(odomFrame,baseLinkFrame,
                              ros::Time(0), tr_o);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        ofs << tr_o.getOrigin() << " " << tr_o.getRotation() << " ";
        ROS_INFO_STREAM("posunutie basu" << tr_o.getOrigin());

        try{
            t.lookupTransform(kinectFrame, worldFrame,
                              ros::Time(0), tr_i);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        ofs << tr_i.getOrigin() << " " << tr_i.getRotation() << endl;

        ROS_INFO_STREAM("posunutie kinectu" << tr_i.getOrigin());

        ROS_INFO_STREAM("PRECITAL SOM");
    }
    ofs.close();

    return 0;
}
