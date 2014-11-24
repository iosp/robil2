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

/**** Old Version //

void ProjectDepthImage(HeightMap* map, Mat img, Vec3D myRight, Vec3D myFront, Vec3D myUp, Vec3D myPos, vector<lane> lanes)
{
  static const double fov = 0.6981317; //45 deg to each side
  double min=100, max=-100;
  double sin_fov = sin(fov);
  double tan_fov = tan(fov);
  //printf("\n\n\nrows: %d\n\n\n\n", img.rows);
  for(int i = img.rows-150; i >= 0; i--)
  {
    for(int j = 0; j < img.cols; j++)
    {
      uchar depth = img.at<uchar>(i, j);
      if(depth < 40 || depth > 150) continue;
      float depth_m = pow(float(depth)/753.42, -1.0661);
      //depth_m = pow(depth, 0.5);
      float right_m = depth_m * sin_fov * 2*(-j + img.cols/2)/img.cols;
      float up_m = depth_m * sin_fov * 2*(-i + img.rows/2)/img.rows;
      Vec3D pos = myPos.add(myFront.multiply(depth_m).add(myRight.multiply(right_m).add(myUp.multiply(up_m))));
      map->setAbsoluteHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));
      if(pos.z > max) max = pos.z;
      if(pos.z < min) min = pos.z;
    }
  }
}

//*/



/**
 * Walrus changes:
 */
//*/
void ProjectDepthImage(HeightMap* map, Mat img, Vec3D myRight, Vec3D myFront, Vec3D myUp, Vec3D myPos, vector<lane> lanes)
{

//   printf("Starting ProjectDepthImage\n");

  static const double fov = 0.6981317; //45 deg to each side
  double min=100, max=-100;
  double sin_fov = sin(fov);
  double tan_fov = tan(fov);
  
  double propotion = LANES_IMAGE_HEIGHT/img.rows;
  
  for(int i = img.rows-1; i >= 0; i--)
  {
    bool mid_road = false;
    bool right_road = false;
    bool left_road = false;
    bool not_road = false;
    bool is_road = false;
    vector<int> lanes_Xs = vector<int>(lanes.size());
    int count = 0;

    for(vector<lane>::iterator it = lanes.begin(); it != lanes.end(); it++)
    {
      //*****Check why function can't get to this point*****//
//       printf("Initiating lanes Xs vector\n");

      lane l = *it;
      if(l.x0 != 0 || l.x1 != 0 || l.x2 != 0)
        lanes_Xs[count] = (int)((l.x2*i*propotion*i*propotion + l.x1*i*propotion + l.x0)/propotion);
      else 
        lanes_Xs[count] = -1;
      count++;
    }
    
    for(int j = 0; j < img.cols; j++)
    {
      if(i > (img.rows)*0 && lanes_Xs.size() > 0)
      {
	if( j < lanes_Xs[0] || j > lanes_Xs.at(count-1))
	{
	  not_road = true;
	}
	if( j > lanes_Xs[0] && j < lanes_Xs.at(count-1) )
	{
	  
	}
	if(lanes_Xs[0] == j)
	{
	  left_road = true;
	}
	else if(lanes_Xs[count-1]  == j)
	{
	  right_road = true;
	}  
	else
	{
	  for(int k=1; k<lanes_Xs.size()-1; k++)
	    if(lanes_Xs[k] == j)
	    mid_road = true;	   
	}
      }
      uchar depth = img.at<uchar>(i, j);
      if(depth < 40 || depth > 150) continue;
      float depth_m = pow(float(depth)/753.42, -1.0661);
      //depth_m = pow(depth, 0.5);
      float right_m = depth_m * sin_fov * 2*(-j + img.cols/2)/img.cols;
      float up_m = depth_m * sin_fov * 2*(-i + img.rows/2)/img.rows;
      Vec3D pos = myPos.add(myFront.multiply(depth_m).add(myRight.multiply(right_m).add(myUp.multiply(up_m))));
      map->setAbsoluteHeightAt((int)(5*pos.x), (int)(5*pos.y), (pos.z));
      
      if(is_road) map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_ROAD);
      if(mid_road) map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_LANE);
      if(left_road) map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_LEFT);
      if(right_road) map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_RIGHT);      
      if(not_road &&
	   (map->getAbsoluteTypeAt((int)(5*pos.x), (int)(5*pos.y)) == TYPE_CLEAR || 
	    map->getAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y)) == FEATURE_ROAD) 
	) 
		map->setAbsoluteFeatureAt((int)(5*pos.x), (int)(5*pos.y), FEATURE_NOT_ROAD);
      if(pos.z > max) max = pos.z;
      if(pos.z < min) min = pos.z;
	 
      mid_road = false;
      left_road = false;
      right_road = false;
      not_road = false;
      is_road = false;
    }
  }
}
//*/
/** Until Here: ********************/


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
