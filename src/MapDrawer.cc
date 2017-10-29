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

#include "MapDrawer.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>
#include <mutex>



namespace ORB_SLAM2
{

static const int HMAP_WIDE=100;
static const int HMAP_DEEP=100;
static const float HMAP_SNAPXZ=1;
static const float HMAP_INVALID=-100;
static float hmap[HMAP_DEEP][HMAP_WIDE];

static std::vector<cv::Vec3f> s_waypoints;

static cv::Vec3f carpos=cv::Vec3f(0,0,0);



void hmap_clear()
{
	for( int jj=0; jj<HMAP_DEEP; jj++ )
	{
		for( int ii=0; ii<HMAP_WIDE; ii++ )
		{
			float yy=HMAP_INVALID;
			if( ii==0 || ii==(HMAP_WIDE-1) || jj==0 || jj==(HMAP_DEEP-1) ){ yy=0; }
			hmap[jj][ii]=yy;
		}
	}
}

float hmap_neighbor_max( int xi, int zi )
{
	float maxyy=HMAP_INVALID;
	for( int jj=zi-1; jj<=zi+1; jj++ )
	{
		for( int ii=xi-1; ii<=xi+1; ii++ )
		{
			if( ii==xi && jj==zi ){ continue; }
			if( ii<0 || ii>=HMAP_WIDE || jj<0 || jj>=HMAP_DEEP ){ continue; }
			float yy=hmap[jj][ii];
			if( yy==HMAP_INVALID ){ continue; }
			if( yy>maxyy )
			{
				maxyy=yy;
			}
		}
	}
	return maxyy;
}

void hmap_set( float xx, float yy, float zz )
{
	xx+=HMAP_SNAPXZ*(HMAP_WIDE*0.5);
	zz+=HMAP_SNAPXZ*(HMAP_DEEP*0.5);
	
	int xi=int(xx/HMAP_SNAPXZ);
	int zi=int(zz/HMAP_SNAPXZ);

	if( xi<0 || xi>=HMAP_WIDE || zi<0 || zi>=HMAP_DEEP ){ return; }

	float yyo=hmap[zi][xi];
	if( yyo==HMAP_INVALID || yy>yyo )
	{
		hmap[zi][xi]=yy;
	}
}

void hmap_draw()
{
	float zz=-HMAP_SNAPXZ*(HMAP_DEEP*0.5);
	for( int zi=0; zi<HMAP_DEEP; zi++ )
	{
		float xx=-HMAP_SNAPXZ*(HMAP_WIDE*0.5);
		for( int xi=0; xi<HMAP_WIDE; xi++ )
		{
			float yy=hmap[zi][xi];
			if( yy!=HMAP_INVALID )
			{
				float tall=0.1;
				float maxyy=hmap_neighbor_max( xi, zi );
				if( maxyy!=HMAP_INVALID ){ tall=maxyy-yy; }
				if( tall<0 ){ tall=0; }

				glPushMatrix();
				glTranslatef( xx, yy+(tall/2), zz);				
				glScalef(HMAP_SNAPXZ/1.0,tall/1.0,HMAP_SNAPXZ/1.0);
				pangolin::glDrawColouredCube(-0.5,0.5);
				glPopMatrix();
			}
			xx+=HMAP_SNAPXZ;
		}
		zz+=HMAP_SNAPXZ;
	}
}



MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath):mpMap(pMap)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    mPointSize = fSettings["Viewer.PointSize"];
    mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
}

void MapDrawer::FollowKeyFrames( unsigned char& out_throttleServo, unsigned char& out_steeringServo )
{
	float maxperpdir=0;
	cv::Vec3f waypos;
	cv::Vec3f wayerr;
	float wayerror=0;

	cv::Mat Tcwi=mCameraPose.inv();
	float camposx=Tcwi.at<float>(0,3);
	float camposy=Tcwi.at<float>(1,3);
	float camposz=Tcwi.at<float>(2,3);

	const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	for( size_t ii=0; ii<vpKFs.size(); ii++ )
	{
		KeyFrame* kf=vpKFs[ii];

		cv::Mat keyT=kf->GetCameraCenter();
		float keyposx=keyT.at<float>(0);
		float keyposy=keyT.at<float>(1);
		float keyposz=keyT.at<float>(2);
		cv::Mat keyR=kf->GetRotation();
		float dirx=keyR.at<float>(2,0);
		float diry=keyR.at<float>(2,1);
		float dirz=keyR.at<float>(2,2);
		float sidex=keyR.at<float>(0,0);
		float sidey=keyR.at<float>(0,1);
		float sidez=keyR.at<float>(0,2);

		float distsqr=	(keyposx-camposx)*(keyposx-camposx)+
						(keyposy-camposy)*(keyposy-camposy)+
						(keyposz-camposz)*(keyposz-camposz);
		
		float perpdir= ((keyposx-camposx)*dirx+(keyposy-camposy)*diry+(keyposz-camposz)*dirz);

		if( distsqr>0.2*0.2 && distsqr<1.5*1.5 && perpdir>maxperpdir )
		{
			waypos=cv::Vec3f( keyposx, keyposy, keyposz );
			maxperpdir=perpdir;

			float perpside=-((waypos[0]-camposx)*sidex+(waypos[1]-camposy)*sidey+(waypos[2]-camposz)*sidez);	
			wayerr[0]=sidex*perpside;
			wayerr[1]=sidey*perpside;
			wayerr[2]=sidez*perpside;
			wayerror=perpside;
		}

	}

	out_throttleServo=90;	
	out_steeringServo=90;

	if( maxperpdir>0 )
	{
		glPointSize(15);
		glBegin(GL_POINTS);
		glColor3f(0.0,1.0,0.0);
		glVertex3f( waypos[0], waypos[1], waypos[2] );
		glEnd();

		glLineWidth(2);
		glBegin(GL_LINES);
		glColor3f(0.0,0.0,1.0);
		glVertex3f( waypos[0], waypos[1], waypos[2] );
		glVertex3f( waypos[0]+wayerr[0], waypos[1]+wayerr[1], waypos[2]+wayerr[2] );
		glEnd();

		
		float maxerror=1;
		float error=2*(wayerror/maxerror);
		if( error>1 ){ error=1; }
		else
		if( error<-1 ){ error=-1; }

		int errori=90+int(error*90);
		if( errori<0 ){ errori=0; }
		if( errori>180 ){ errori=180; }

		out_steeringSestd_msgs/UInt16rvo=(unsigned char)(errori&255);
	}

}

void MapDrawer::DrawWaypoints( bool generate, bool lower )
{
	float yoff=0;
	if( lower ){ yoff+=0.4; }

	pangolin::OpenGlMatrix oglcamM;
	GetCurrentOpenGLCameraMatrix(oglcamM);
	float camx=oglcamM.m[12];
	float camy=oglcamM.m[13];
	float camz=oglcamM.m[14];

	if( s_waypoints.size()==0 )
	{
		s_waypoints.push_back( cv::Vec3f( camx, camy, camz ) );
	}

	float posx=s_waypoints[s_waypoints.size()-1][0];
	float posy=s_waypoints[s_waypoints.size()-1][1];
	float posz=s_waypoints[s_waypoints.size()-1][2];

	float distsqr=  (posx-camx)*(posx-camx)+
					(posy-camy)*(posy-camy)+
					(posz-camz)*(posz-camz);
	if( generate && distsqr>1*1 )
	{
		s_waypoints.push_back( cv::Vec3f( camx, camy, camz ) );
	}

	glLineWidth(2);
	glBegin(GL_LINE_STRIP);
	glColor3f(0.0,1.0,0.0);
	for( int ii=0, iiend=s_waypoints.size(); ii<iiend; ii++ )
	{
		posx=s_waypoints[ii][0];
		posy=s_waypoints[ii][1];
		posz=s_waypoints[ii][2];
		glVertex3f( posx, posy+yoff, posz );
	}
	//glVertex3f( camx, camy, camz );
	glEnd();

	glPointSize(10);
	glBegin(GL_POINTS);
	glColor3f(1.0,1.0,1.0);
	for( int ii=0, iiend=s_waypoints.size()-1; ii<iiend; ii++ )
	{
		posx=s_waypoints[ii][0];
		posy=s_waypoints[ii][1];
		posz=s_waypoints[ii][2];
		glVertex3f( posx, posy+yoff, posz );
	}
	glEnd();

}

void MapDrawer::DrawMapPoints(bool drawPoints, bool drawTiles )
{
	hmap_clear();
	
/*
	const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();
    const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

    set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

    if(vpMPs.empty())
		return;
		
	if( drawPoints )
	{
    glPointSize(mPointSize*2);
    glBegin(GL_POINTS);
    glColor3f(0.0,1.0,0.0);
	}

    for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
    {
        if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i])){ continue; }
        cv::Mat pos = vpMPs[i]->GetWorldPos();
		
		if( drawPoints )
		glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
	
		if( drawTiles )
		if(vpMPs[i]->Observations()>=4)
		{hmap_set(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));}
	}
    if( drawPoints )glEnd();

	if( drawPoints )
	{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,0.0,0.0);
	}

    for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
    {
        if((*sit)->isBad()){ continue; }
        cv::Mat pos = (*sit)->GetWorldPos();
		
		if( drawPoints )
		glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

		if( drawTiles )
		if((*sit)->Observations()>=4)
		{hmap_set(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));}
    }

    if( drawPoints )glEnd();
*/	

	


	if( drawPoints )
	{
    glPointSize(mPointSize);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
	}
	const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
	for(size_t jj=0; jj<vpKFs.size(); jj++)
	{
		KeyFrame* kf=vpKFs[jj];
		const vector<MapPoint*> &vpMPs=kf->GetMapPointMatches();

		cv::Mat campos=kf->GetCameraCenter();
		float camposx=campos.at<float>(0);
		float camposy=campos.at<float>(1);
		float camposz=campos.at<float>(2);
		cv::Mat camR=kf->GetRotation();
		float upx=camR.at<float>(0,1);
		float upy=camR.at<float>(1,1);
		float upz=camR.at<float>(2,1);
		//printf("%f %f %f\n",upx,upy,upz);

		set<MapPoint*> spRefMPs=kf->GetMapPoints();
		for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
		{
			if((*sit)->isBad()){ continue; }
			cv::Mat pos = (*sit)->GetWorldPos();


		//for(size_t ii=0, iiend=vpMPs.size(); ii<iiend;ii++)
    	//{
			//if( vpMPs[ii]==NULL){ continue; }
        	//if( vpMPs[ii]->isBad() ){ continue; }
			
			//cv::Mat pos = vpMPs[ii]->GetWorldPos();
			float posx=pos.at<float>(0);
			float posy=pos.at<float>(1);
			float posz=pos.at<float>(2);

			/*
			float distsqr=  (posx-camposx)*(posx-camposx)+
							(posy-camposy)*(posy-camposy)+
							(posz-camposz)*(posz-camposz);
			*/
			bool lower=true;
			glColor3f(1.0,1.0,1.0);

			float dot=(posx-camposx)*upx+(posy-camposy)*upy+(posz-camposz)*upz;

			if(dot<0){ lower=false; glColor3f(1.0,0.0,0.0); }
			
			if( drawPoints )
			glVertex3f(posx,posy,posz);
	
			if( drawTiles && lower ) //&& distsqr>1*1 && distsqr<10*10 )
			if( (*sit)->Observations()>=5)
			//if( vpMPs[ii]->GetFound()>10 )
			{hmap_set(posx,posy,posz);}
		}
	}
	if( drawPoints )
	glEnd();


	if( drawTiles )hmap_draw();



	/*
	for( int ii=0, iiend=s_waypoints.size()-1; ii<iiend; ii++ )
	{
		cv::Vec3f waya=s_waypoints[ii+0];
		cv::Vec3f wayb=s_waypoints[ii+1];

		cv::Vec3f waydir=cv::Vec3f(wayb[0]-waya[0],wayb[1]-waya[1],wayb[2]-waya[2]);
		float waydirlensqr=waydir[0]*waydir[0]+waydir[1]*wadir[1]+waydir[2]*wadir[2];
		if( waydirlensqr>0.01 ){ continue; }
		float waydirlen=sqrt(waydirlensqr);
		waydir[0]/=waydirlen;
		waydir[1]/=waydirlen;
		waydir[2]/=waydirlen;

		cv::Vec3f test=carpos;
		test[0]-=waya[0];
		test[1]-=waya[1];
		test[2]-=waya[2];
		float dot=test[0]*waydir[0]+test[1]*waydir[1]+test[2]*waydir[2];
		
		test[0]+=dot*waydir[0];
		test[1]+=dot*waydir[1];
		test[2]+=dot*waydir[2];

	}
	*/


}

void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
{
    const float &w = mKeyFrameSize;
    const float h = w*0.75;
    const float z = w*0.6;

    const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

    if(bDrawKF)
    {
        for(size_t i=0; i<vpKFs.size(); i++)
        {
            KeyFrame* pKF = vpKFs[i];
            cv::Mat Twc = pKF->GetPoseInverse().t();

            glPushMatrix();

            glMultMatrixf(Twc.ptr<GLfloat>(0));

            glLineWidth(mKeyFrameLineWidth*2);
            glColor3f(1.0f,1.0f,1.0f);
            glBegin(GL_LINES);
            glVertex3f(0,0,0);
            glVertex3f(w,h,z);
            glVertex3f(0,0,0);
            glVertex3f(w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,-h,z);
            glVertex3f(0,0,0);
            glVertex3f(-w,h,z);

            glVertex3f(w,h,z);
            glVertex3f(w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(-w,-h,z);

            glVertex3f(-w,h,z);
            glVertex3f(w,h,z);

            glVertex3f(-w,-h,z);
            glVertex3f(w,-h,z);
            glEnd();

            glPopMatrix();
        }
    }

    if(bDrawGraph)
    {
        glLineWidth(mGraphLineWidth);
        glColor4f(0.0f,1.0f,0.0f,0.6f);
        glBegin(GL_LINES);

        for(size_t i=0; i<vpKFs.size(); i++)
        {
            // Covisibility Graph
            const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
            cv::Mat Ow = vpKFs[i]->GetCameraCenter();
            if(!vCovKFs.empty())
            {
                for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                {
                    if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                }
            }

            // Spanning tree
            KeyFrame* pParent = vpKFs[i]->GetParent();
            if(pParent)
            {
                cv::Mat Owp = pParent->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
            }

            // Loops
            set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
            for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
            {
                if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                cv::Mat Owl = (*sit)->GetCameraCenter();
                glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
            }
        }

        glEnd();
    }
}

void MapDrawer::DrawCurrentCamera()
{
	pangolin::OpenGlMatrix Twc;
	GetCurrentOpenGLCameraMatrix(Twc);


    const float &w = mCameraSize;
    const float h = w*0.75;
    const float z = w*0.6;

    glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

    glLineWidth(mCameraLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}


void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
{
    unique_lock<mutex> lock(mMutexCamera);
    mCameraPose = Tcw.clone();
}

void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
{
    if(!mCameraPose.empty())
    {
        cv::Mat Rwc(3,3,CV_32F);
        cv::Mat twc(3,1,CV_32F);
        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
        }

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}


} //namespace ORB_SLAM
