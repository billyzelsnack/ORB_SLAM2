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

#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>


namespace ORB_SLAM2
{

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath):
    mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    mImageWidth = fSettings["Camera.width"];
    mImageHeight = fSettings["Camera.height"];
    if(mImageWidth<1 || mImageHeight<1)
    {
        mImageWidth = 640;
        mImageHeight = 480;
    }

    mViewpointX = 0;//fSettings["Viewer.ViewpointX"];
    mViewpointY = -0.07;//fSettings["Viewer.ViewpointY"];
    mViewpointZ = -0.0018;//fSettings["Viewer.ViewpointZ"];
    mViewpointF = 500;//fSettings["Viewer.ViewpointF"];
}

void Viewer::Run()
{
	int w,h;
	cv::Mat im, Tcw;
	int status;
	vector<cv::KeyPoint> vKeys;
	vector<MapPoint*> vMPs;

	while(1)
    {
        GetImagePose(im,Tcw,status,vKeys,vMPs);
        if(im.empty())
            cv::waitKey(mT);
        else
        {
            w = im.cols;
            h = im.rows;
            break;
        }
    }



    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("Jeep hmap",752*2,480*2);//1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.2,250),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
				);

		float fx = 448.803530;
		float fy = 448.803530;
		float cx = 366.972919;
		float cy = 242.598818;				
		pangolin::OpenGlMatrixSpec P =pangolin::ProjectionMatrixRDF_TopLeft( w,h,fx,fy,cx,cy, 0.01,1000 );
				



		pangolin::View& d_image = pangolin::Display("image")
		.SetBounds(0,1.0f,  0 ,1.0f,(float)w/h)
		.SetLock(pangolin::LockLeft, pangolin::LockTop);


	
    	pangolin::View& d_cam = pangolin::Display("cam")
            //.SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -w/h)
			.SetBounds(0,1.0f,  0 ,1.0f,(float)w/h)
			.SetHandler(new pangolin::Handler3D(s_cam));


			pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
			pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
			//pangolin::Var<bool> menuShowTiles("menu.Show Tiles",false,true);
			pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
			pangolin::Var<bool> menuShowWaypoints("menu.Show Waypoints",true,true);
			pangolin::Var<bool> menuShowWaypointsLower("menu.Show Waypoints Lower",true,true);
			pangolin::Var<bool> menuGenerateWaypoints("menu.Generate Waypoints",false,true);
			pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",false,true);
			pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
			pangolin::Var<bool> menuFollowKeyFrames("menu.Follow KeyFrames",false,true);			
			pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
			pangolin::Var<bool> menuReset("menu.Reset",false,false);
	
	pangolin::RegisterKeyPressCallback(' ', 
	pangolin::ToggleVarFunctor("menu.Follow Camera"));
			
	pangolin::GlTexture imageTexture(w,h,GL_RGBA8,false,0,GL_RGB ,GL_UNSIGNED_BYTE);
			


    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    //cv::namedWindow("ORB-SLAM2: Current Frame");

    //bool bFollow = true;
	bool bLocalizationMode = false;



	//int dropctr=0;
	//std::vector<cv::Vec3f> drops;
	
    while(1)
    {
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		


		//mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);
		

		//bFollow=menuFollowCamera;

		/*
		if(menuFollowCamera && bFollow)
        {
            s_cam.Follow(Twc);
        }
        else if(menuFollowCamera && !bFollow)
        {
			
            //s_cam.SetModelViewMatrix(
			//	//pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0, 0.0,-1.0, 0.0)
			//	pangolin::ModelViewLookAt( 0.0f, 0.0f, 0.0f,  -1.0f, 0.0f, 0.0f,  0.0f, -1.0f, 0.0f )
			//	//pangolin::IdentityMatrix()
			//);
			
			
            s_cam.Follow(Twc);
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
		}
		*/

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
		}


		glClearColor(0,0,0,1);//1.0f,1.0f,1.0f,1.0f);

		
		

		d_image.Activate();
        glColor3f(1.0,1.0,1.0);
		
		//glMatrixMode(GL_PROJECTION);
        //P.Load();
		//glMatrixMode(GL_MODELVIEW);


		
		GetImagePose(im,Tcw,status,vKeys,vMPs);
		cv::Mat Tcwi=Tcw.inv();
		if(menuFollowCamera && !Tcw.empty())
		{
			pangolin::OpenGlMatrix M;
	
			M.m[0] = Tcw.at<float>(0,0);
			M.m[1] = Tcw.at<float>(1,0);
			M.m[2] = Tcw.at<float>(2,0);
			M.m[3]  = 0.0;
	
			M.m[4] = Tcw.at<float>(0,1);
			M.m[5] = Tcw.at<float>(1,1);
			M.m[6] = Tcw.at<float>(2,1);
			M.m[7]  = 0.0;
	
			M.m[8] = Tcw.at<float>(0,2);
			M.m[9] = Tcw.at<float>(1,2);
			M.m[10] = Tcw.at<float>(2,2);
			M.m[11]  = 0.0;
	
			M.m[12] = Tcw.at<float>(0,3);
			M.m[13] = Tcw.at<float>(1,3);
			M.m[14] = Tcw.at<float>(2,3);
			M.m[15]  = 1.0;
	
			//M.Load();

			s_cam.SetProjectionMatrix( P );
			s_cam.SetModelViewMatrix( M );

			//s_cam.Follow(Twc);
			string statusText="dunno";
			switch( status )
			{
				case Tracking::SYSTEM_NOT_READY:{ statusText="SYSTEM_NOT_READY"; break; }
				case Tracking::NO_IMAGES_YET:{ statusText="NO_IMAGES_YET"; break; }
				case Tracking::NOT_INITIALIZED:{ statusText="NOT_INITIALIZED"; break; }
				case Tracking::OK:{ statusText="OK"; break; }
				case Tracking::LOST:{ statusText="LOST"; break; }
			}

			cv::putText(im,statusText,cv::Point(100,im.rows-3*20),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,255,255),1,8);			
			cv::putText(im,string("ThrottleServo: ")+std::to_string(mpSystem->throttleServo),cv::Point(100,im.rows-2*20),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,255,255),1,8);						
			cv::putText(im,string("SteeringServo: ")+std::to_string(mpSystem->steeringServo),cv::Point(100,im.rows-1*20),cv::FONT_HERSHEY_PLAIN,1.0,cv::Scalar(255,255,255),1,8);			
			
			imageTexture.Upload(im.data,GL_LUMINANCE,GL_UNSIGNED_BYTE);
			imageTexture.RenderToViewportFlipY();
		}


		glClear(GL_DEPTH_BUFFER_BIT);
		
		
		d_cam.Activate(s_cam);
		if( !menuFollowCamera )
		{ mpMapDrawer->DrawCurrentCamera( ); }

		if( menuShowWaypoints ){ mpMapDrawer->DrawWaypoints( menuGenerateWaypoints, menuShowWaypointsLower ); }

        if(menuShowKeyFrames || menuShowGraph)
            mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph);
		
		
		if(menuShowPoints)//||menuShowTiles)
			mpMapDrawer->DrawMapPoints(menuShowPoints,false);//menuShowTiles);

		if(menuFollowKeyFrames){ mpMapDrawer->FollowKeyFrames( mpSystem->throttleServo, mpSystem->steeringServo ); }

			/*
			const int N = vKeys.size();
			
			
				for(int i=0; i<N; i++)
				{
					if(vMPs[i])
					{
						cv::circle(im,vKeys[i].pt,1,cv::Scalar(0,255,0),-1);
					}
				}
			*/
			

		/*
		//if( dropctr<=0 )
		{
			dropctr=100;

			float camx=Tcwi.at<float>(0,3);
			float camy=Tcwi.at<float>(1,3);
			float camz=Tcwi.at<float>(2,3);

			//printf("camy %f.2\n",camy);
			
			int mindistindex=-1;
			float mindistsqr=1000*1000;
			for( size_t ii=0, iicount=vMPs.size() ; ii<iicount; ii++ )
			{
				if(!vMPs[ii]){continue;}
				if(vMPs[ii]->isBad()){continue;}
				if(vMPs[ii]->Observations()<6){continue;}

				cv::Mat pos=vMPs[ii]->GetWorldPos();
				float xx=pos.at<float>(0);
				float yy=pos.at<float>(1);
				float zz=pos.at<float>(2);

				if( (yy-camy)<0 ){ continue; }

				float dist=(xx-camx)*(xx-camx)+
						   //(yy-camy)*(yy-camy)+
						   (zz-camz)*(zz-camz);
				//if( ( mindistindex==-1 || dist<mindistsqr ) && dist<(4*4) ){ mindistindex=ii; mindistsqr=dist; }
			
				if( dist<(3*3) ) { drops.push_back(cv::Vec3f(xx,yy,zz)); }
			}


			//cv::Vec3f(Tcwi.at<float>(0,3),Tcwi.at<float>(1,3),Tcwi.at<float>(2,3) ));
		}
		dropctr--;

		
		for( size_t ii=0; ii<drops.size(); ii++ )
		{
			float snap=1;
			float tall=0.1;

			float xx=snap*round( drops[ii][0]/snap );
			float zz=snap*round( drops[ii][2]/snap );

			pangolin::OpenGlMatrix M = pangolin::OpenGlMatrix::Translate(xx,drops[ii][1]+(tall/2),zz);
			glPushMatrix();
			M.Multiply();
			glScalef(snap/1,tall/1,snap/1);
			pangolin::glDrawColouredCube(-0.5,0.5);
			glPopMatrix();
		}
		*/
		
        pangolin::FinishFrame();

        //cv::Mat im = mpFrameDrawer->DrawFrame();
        //cv::imshow("ORB-SLAM2: Current Frame",im);
        //cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = false;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            //bFollow = true;
            menuFollowCamera = true;
            mpSystem->Reset();
            menuReset = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
	}

    SetFinish();
}

void Viewer::SetImagePose(const cv::Mat &im, const cv::Mat &Tcw, const int &status, const vector<cv::KeyPoint> &vKeys, const vector<ORB_SLAM2::MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    mImage = im.clone();
    mTcw = Tcw.clone();
    mStatus = status;
    mvKeys = vKeys;
	mvMPs = vMPs;
	
}

void Viewer::GetImagePose(cv::Mat &im, cv::Mat &Tcw, int &status, std::vector<cv::KeyPoint> &vKeys,  std::vector<MapPoint*> &vMPs)
{
    unique_lock<mutex> lock(mMutexPoseImage);
    im = mImage.clone();
    Tcw = mTcw.clone();
    status = mStatus;
    vKeys = mvKeys;
    vMPs = mvMPs;
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

}
