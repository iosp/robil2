#ifndef STEREO__H
#define STEREO__H
#include <boost/thread/thread.hpp>
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
    //CvMat * disp_left_visual= cvCreateMat(temp.height, temp.width, CV_8U);
    //cvConvertScale( matf, disp_left_visual, -16 );
    cvNormalize( matf, matf, 0, 255, CV_MINMAX, NULL );
    int i, j;
    uchar *ptr_dst;
    IplImage *cv_image_depth_aux = cvCreateImage (cvGetSize(&temp),IPL_DEPTH_8U, 1);
    for ( i = 0; i < matf->rows; i++)
    {
    	ptr_dst = (uchar*)(cv_image_depth_aux->imageData + i*cv_image_depth_aux->widthStep);
    	for ( j = 0; j < matf->cols; j++ )
    	{
    		ptr_dst[j] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		//ptr_dst[3*j+1] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    		//ptr_dst[3*j+2] = (uchar)((short int*)(matf->data.ptr + matf->step*i))[j];
    	}
    }
    //system("pause");
    //cvSaveImage("disp.ppm", &cv_image_depth_aux);
    //system("pause");
    Mat img(cv_image_depth_aux, true);
	//system("pause");
    //cvReleaseImage(&cv_image_depth_aux);
    cvReleaseMat(&matf);
    //cvReleaseMat(&disp_left_visual);
   // cvReleaseImage(temp);
    //cvReleaseImage(temp2);
    cvReleaseStereoBMState(&state);
	
    //cvtColor(img,img, CV_BGR2GRAY);
    
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


boost::mutex mutex;

void ProjectDepthImage(HeightMap* map, Mat img, Vec3D myRight, Vec3D myFront, Vec3D myUp, Vec3D myPos, Mat lanes)
{
  return;
  mutex.lock();
  static const double fov = 0.6981317; //45 deg to each side
  double min=100, max=-100;
  double sin_fov = sin(fov);
  double tan_fov = tan(fov);
  
  bool emptyLanes = lanes.empty();
    
  cv::resize(lanes, lanes, img.size());

  bool not_road = false;
  for(int i = 0; i <img.rows; i++)
  {
    for(int j = 0; j < img.cols; j++)
    {
      if(!emptyLanes && lanes.at<Vec3b>(i,j) == Vec3b(0,0,0))
      {
	not_road = true;
      }
      
      uchar depth = img.at<uchar>(i, j);
      //if(depth < 40 || depth > 150) continue;
      float depth_m = pow(float(depth)/753.42, -1.0661);
      //depth_m = pow(depth, 0.5);
      float right_m = depth_m * sin_fov * 2*(-j + img.cols/2)/img.cols;
      float up_m = depth_m * sin_fov * 2*(-i + img.rows/2)/img.rows;
      Vec3D pos = myPos.add(myFront.multiply(depth_m).add(myRight.multiply(right_m).add(myUp.multiply(up_m))));
//       map->setAbsoluteHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));
           
      
      if(pos.z > max) max = pos.z;
      if(pos.z < min) min = pos.z;
      
      if(not_road &&
	   (map->getAbsoluteTypeAt((int)(5*pos.x), (int)(5*pos.y)) == TYPE_CLEAR || 
	    map->getAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y)) == FEATURE_ROAD) 
	) 
	map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_NOT_ROAD);

      not_road = false;
    }
  }
    
//   if(!emptyLanes)
//     imshow("after", lanes);
//     waitKey(1);
  mutex.unlock();
}

Mat handleStereo(Mat left, Mat right)
{
  Mat l,r;
  resize(left, l, Size(left.size().width/2, left.size().height/2), 0, 0, cv::INTER_CUBIC);
  resize(right, r, Size(right.size().width/2, right.size().height/2), 0, 0, cv::INTER_CUBIC);
  
  //StereoVar sv();
  //Mat stereo;
  //sv.operator()(l,r,stereo);
  
  Mat stereo = getDisparity(l, r);
  //stereo = Mat(stereo, Rect(60, 0, stereo.cols-61, stereo.rows-1));
  //fastNlMeansDenoising(stereo, stereo);
  Mat median;
  medianBlur(stereo, median, 15);
  //stereo = filterDisparity(stereo);
  
  bitwise_and(stereo, median, stereo);
  //imshow("stereo", stereo); 
  
  //imshow("asdf", median); 
  //imshow("L", l); 
  //imshow("R", r);
  //waitKey(1);
  //waitKey(1);
  return stereo;
}




#endif
