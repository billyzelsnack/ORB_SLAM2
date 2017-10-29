/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/UInt16.h>

#include <tf/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/highgui.hpp>

//#include<opencv2/core/core.hpp>

//#include"../../../include/System.h"
#include "../include/System.h"
#include "../include/Tracking.h"

using namespace std;





class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    ORB_SLAM2::System* mpSLAM;
    bool do_rectify;
    cv::Mat M1l,M2l,M1r,M2r;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "orbslam2_stereo_node");

    if(argc != 5)
    {
        cerr << endl << "Usage: rosrun orbslam2_ros orbslam2_stereo_node path_to_vocabulary path_to_settings do_rectify" << endl;
        ros::shutdown();
        return 1;
    }    

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[2],argv[3],ORB_SLAM2::System::STEREO,true);

    ImageGrabber igb(&SLAM);

    stringstream ss(argv[4]);
	ss >> boolalpha >> igb.do_rectify;

    if(igb.do_rectify)
    {      
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(argv[3], cv::FileStorage::READ);
        if(!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            return -1;
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
                rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            return -1;
        }

        cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,igb.M1l,igb.M2l);
        cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,igb.M1r,igb.M2r);
    }

	ros::NodeHandle nh;
	
	ros::Publisher throttleServo_pub =nh.advertise<std_msgs::UInt16> ( "/throttleServo", 1 );
	ros::Publisher steeringServo_pub =nh.advertise<std_msgs::UInt16> ( "/steeringServo", 1 );
	

	/*
	ros::Publisher odometry_pub = nh.advertise<nav_msgs::Odometry> ( "/odometry", 1 );
	{
	odometry_pub.header.seq=
	odometry_pub.header.stamp=
	odometry_pub.header.frame_id=
	odometry_pub.child_frame_id;
	
	geometry_msgs::PoseWithCovariance posecov;
	posecov.pose.position.x=
	posecov.pose.position.y=
	posecov.pose.position.z=
	posecov.orientation.x=
	posecov.orientation.y=
	posecov.orientation.z=
	posecov.orientation.w=
	posecov.covariance={cox_x, 0, 0, 0, 0, 0,
						0, cov_y, 0, 0, 0, 0,
						0, 0, cov_z, 0, 0, 0,
						0, 0, 0, 99999, 0, 0,
						0, 0, 0, 0, 99999, 0,
						0, 0, 0, 0, 0, 99999}
	odometry_pub.pose=posecov;

	geometry_msgs::TwistWithCovariance twistcov;
	twistcov.twist.linear.x=
	twistcov.twist.linear.y=
	twistcov.twist.linear.z=
	twistcov.twist.angular.x=
	twistcov.twist.angular.y=
	twistcov.twist.angular.z=	
	twistcov.covariance={cox_x, 0, 0, 0, 0, 0,
						0, cov_y, 0, 0, 0, 0,
						0, 0, cov_z, 0, 0, 0,
						0, 0, 0, 99999, 0, 0,
						0, 0, 0, 0, 99999, 0,
						0, 0, 0, 0, 0, 99999}
	odometry_pub.twist=twistcov;
	}
	*/

	//ros::Publisher poseStampedPublisher = nh.advertise<geometry_msgs::PoseStamped> ( "/pose", 1 );	

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub,right_sub);
	sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo,&igb,_1,_2));
	 
	ros::start();
	
	//ros::spin();
	
	//ros::Rate loop_rate(30);
	while( ros::ok() ) 
	{
		ros::spinOnce();

		ORB_SLAM2::Tracking* tracking=SLAM.GetTracking();
		
		if( tracking->mState==ORB_SLAM2::Tracking::OK )
		{
			double timeStamp=tracking->mCurrentFrame.mTimeStamp;			

			/*
			cv::Mat Tcwi=tracking->mCurrentFrame.mTcw.inv();
			tf::Matrix3x3 ori( 	Tcwi.at<float>( 0, 0 ), Tcwi.at<float>( 0, 1 ), Tcwi.at<float>( 0, 2 ),
								Tcwi.at<float>( 1, 0 ), Tcwi.at<float>( 1, 1 ), Tcwi.at<float>( 1, 2 ),
								Tcwi.at<float>( 2, 0 ), Tcwi.at<float>( 2, 1 ), Tcwi.at<float>( 2, 2 ) );
			tf::Quaternion quaternion;
			ori.getRotation(quaternion);
	
			geometry_msgs::PoseStamped poseStamped;
			poseStamped.header.stamp=ros::Time(timeStamp);
			poseStamped.header.frame_id="/world";			
			poseStamped.pose.orientation.x=quaternion.getX();
			poseStamped.pose.orientation.y=quaternion.getY();
			poseStamped.pose.orientation.z=quaternion.getZ();
			poseStamped.pose.orientation.w=quaternion.getW();
			poseStamped.pose.position.x=Tcwi.at<float>( 0, 3 );
			poseStamped.pose.position.y=Tcwi.at<float>( 1, 3 );
			poseStamped.pose.position.z=Tcwi.at<float>( 2, 3 );

			poseStampedPublisher.publish( poseStamped );			
			*/

			throttleServo_pub.publish( SLAM.throttleServo );
			steeringServo_pub.publish( SLAM.steeringServo );
		}


		//loop_rate.sleep();
	}

    // Stop all threads
    SLAM.Shutdown();

    // Save camera trajectory
    //SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryTUM("FrameTrajectory_TUM_Format.txt");
    //SLAM.SaveTrajectoryKITTI("FrameTrajectory_KITTI_Format.txt");

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrLeft;
    try
    {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrRight;
    try
    {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

	cv::Mat Tcw;
	cv::Mat imLeft, imRight;
	//cv::Mat im;	
	if(do_rectify)
    {
        cv::remap(cv_ptrLeft->image,imLeft,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(cv_ptrRight->image,imRight,M1r,M2r,cv::INTER_LINEAR);
		Tcw=mpSLAM->TrackStereo(imLeft,imRight,cv_ptrLeft->header.stamp.toSec());
		//im=imLeft.clone();
	}
    else
    {
        Tcw=mpSLAM->TrackStereo(cv_ptrLeft->image,cv_ptrRight->image,cv_ptrLeft->header.stamp.toSec());
		//im=cv_ptrLeft->image;
	}
	
	if( !Tcw.empty() )//&& !im.empty() )
	{
    	int state = mpSLAM->GetTrackingState();
    	vector<ORB_SLAM2::MapPoint*> vMPs = mpSLAM->GetTrackedMapPoints();
    	vector<cv::KeyPoint> vKeys = mpSLAM->GetTrackedKeyPointsUn();
	
		cv::Mat im=mpSLAM->GetTracking()->mImGray.clone();//copyTo(im);
		//cvtColor(im,im,CV_GRAY2RGB);
		
		mpSLAM->mpViewer->SetImagePose(im,Tcw,state,vKeys,vMPs);
	}

}


