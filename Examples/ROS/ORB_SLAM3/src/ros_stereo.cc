/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include <opencv2/core/core.hpp>

#include "../../../include/System.h"

using namespace std;

ORB_SLAM3::Frame frame = {};
nav_msgs::Odometry odom;

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM3::System *pSLAM) : mpSLAM(pSLAM) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    ORB_SLAM3::System *mpSLAM;
    bool do_rectify;
    cv::Mat M1l, M2l, M1r, M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    // ros::NodeHandle n("orb_slam3");

    ros::start();

    if (argc != 5)
    {
        cerr << endl
             << "Usage: rosrun ORB_SLAM3 Stereo path_to_vocabulary path_to_settings do_rectify enable_view" << endl;
        ros::shutdown();
        return 1;
    }

    std::string sbView(argv[4]);
    bool bView = true;
    if (sbView == "false")
        bView = false;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, bView);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[3]);
    ss >> boolalpha >> igb.do_rectify;

    ros::NodeHandle nh("orb_slam3");

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));

    // ros::spin();

    Eigen::Matrix3f rotm;
    rotm << 0, 1, 0, -1, 0, 0, 0, 0, 1;
    const Eigen::Quaternionf rotq(rotm);

    tf::TransformBroadcaster odom_broadcaster;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    Sophus::SE3f prevTcw;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::Rate loop_rate(200);

    while (nh.ok())
    {
        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

        const Sophus::SE3f Tcw = frame.GetPose().inverse();

        // first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = Tcw.translation()[1];
        odom_trans.transform.translation.y = -Tcw.translation()[0];
        odom_trans.transform.translation.z = Tcw.translation()[2];
        const Eigen::Quaternionf rotted = rotq * Tcw.so3().unit_quaternion();
        odom_trans.transform.rotation.x = rotted.x();
        odom_trans.transform.rotation.y = rotted.y();
        odom_trans.transform.rotation.z = rotted.z();
        odom_trans.transform.rotation.w = rotted.w();

        // send the transform
        odom_broadcaster.sendTransform(odom_trans);

        /**
         * This is a message object. You stuff it with data, and then publish it.
         */

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = Tcw.translation()[1];
        odom.pose.pose.position.y = -Tcw.translation()[0];
        odom.pose.pose.position.z = Tcw.translation()[2];

        odom.pose.pose.orientation.x = rotted.x();
        odom.pose.pose.orientation.y = rotted.y();
        odom.pose.pose.orientation.z = rotted.z();
        odom.pose.pose.orientation.w = rotted.w();

        double dt = (current_time - last_time).toSec();
        const Sophus::SE3f diff = Tcw * prevTcw;

        // odom.twist.twist.linear.x = diff.translation()[1]/dt;
        // odom.twist.twist.linear.y = -diff.translation()[0]/dt;
        // odom.twist.twist.linear.z = diff.translation()[2]/dt;
        odom.twist.twist.linear.x = frame.GetVelocity()[1];
        odom.twist.twist.linear.y = -frame.GetVelocity()[0];
        odom.twist.twist.linear.z = frame.GetVelocity()[2];

        prevTcw = frame.GetPose();

        last_time = current_time;

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        odom_pub.publish(odom);

        loop_rate.sleep();
    }

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (do_rectify)
    {
        cv::Mat imLeft, imRight;
        cv::remap(cv_ptrLeft->image, imLeft, M1l, M2l, cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image, imRight, M1r, M2r, cv::INTER_LINEAR);
        frame = mpSLAM->TrackStereo(imLeft, imRight, cv_ptrLeft->header.stamp.toSec());
    }
    else
    {
        frame = mpSLAM->TrackStereo(cv_ptrLeft->image, cv_ptrRight->image, cv_ptrLeft->header.stamp.toSec());
    }
}
