#ifndef STEREO__H
#define STEREO__H

#include <opencv2/opencv.hpp>
#include "rdbg.h"
#define PI 3.14159

using namespace cv;

Mat getDisparity(Mat left_image, Mat right_image)
{
    cvtColor( left_image,left_image, CV_BGR2GRAY);
    cvtColor( right_image,right_image,CV_BGR2GRAY);
    IplImage temp=left_image;
    IplImage temp2=right_image;
    CvMat *matf= cvCreateMat ( temp.height, temp.width, CV_16S);
    CvStereoBMState * state=cvCreateStereoBMState(CV_STEREO_BM_BASIC,16*4);
    cvFindStereoCorrespondenceBM(&temp,&temp2,matf,state);
    CvMat * disp_left_visual= cvCreateMat(temp.height, temp.width, CV_8U);
    cvConvertScale( matf, disp_left_visual, -16 );
    cvNormalize( matf, matf, 0, 255, CV_MINMAX, NULL );
    int i, j;
    uchar *ptr_dst;
    IplImage *cv_image_depth_aux = cvCreateImage (cvGetSize(&temp),IPL_DEPTH_8U, 3);
    for ( i = 0; i < matf->rows; i++)
    {
    	ptr_dst = (uchar*)(cv_image_depth_aux->imageData + i*cv_image_depth_aux->widthStep);
    	for ( j = 0; j < matf->cols; j++ )
    	{
    		ptr_dst[3*j] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		ptr_dst[3*j+1] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		ptr_dst[3*j+2] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    	}
    }
    //system("pause");
    //cvSaveImage("disp.ppm", &cv_image_depth_aux);
    //system("pause");
    Mat img(cv_image_depth_aux, true);
	//system("pause");
    cvReleaseImage(&cv_image_depth_aux);
	
    cvtColor(img,img, CV_BGR2GRAY);
    
    return img;
}

Mat filterDisparity(Mat img)
{
  static const int thresh = 12;
  for(int i = 0; i < img.rows; i++)
  {
    int follow = 0;
    for(int j = 0; j < img.cols; j++)
    {
      //img.at<uchar>(i,j) = 0;
      uchar depth = img.at<uchar>(i,j);
      if(depth > 0)
      {
	follow++;
      }
      else
      {
	if(follow > 0 && follow < thresh)
	{
	  for(int x = j-1; x >= j-follow; x--)
	    img.at<uchar>(i,x) = 0;
	  follow = 0;
	}
      }
    }
  }
  return img;
}


void ProjectDepthImage(HeightMap* map, Mat img, Vec3D myRight, Vec3D myFront, Vec3D myUp, Vec3D myPos)
{
  static const double asp = PI/4; //45 deg to each side
  for(int i = 0; i < img.rows; i++)
  {
    for(int j = 0; j < img.cols; j++)
    {
      uchar depth = img.at<uchar>(i, j);
      if(depth < 10 || depth > 220) continue;
      float depth_m = (255-(int)depth)/20.0f;
      float right_m = depth_m * tan(asp) * 2*(j - img.cols/2)/img.cols;
      float up_m = depth_m * tan(asp) * 2*(i - img.rows/2)/img.rows;
      Vec3D pos = myPos.add(myFront.multiply(depth_m).add(myRight.multiply(right_m).add(myUp.multiply(up_m))));
      map->setAbsoluteHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));   
    }
  }
  
}

Mat handleStereo(Mat left, Mat right)
{
  Mat l,r;
  resize(left, l, Size(left.size().width/2, left.size().height/2), 0, 0, cv::INTER_CUBIC);
  resize(right, r, Size(right.size().width/2, right.size().height/2), 0, 0, cv::INTER_CUBIC);
 
  Mat stereo = getDisparity(r, l);
  //stereo = filterDisparity(stereo);
  //imshow("dispx", stereo); 
  //imshow("L", l); 
  //imshow("R", r);
  //waitKey(1);
  return stereo;
}



#endif
