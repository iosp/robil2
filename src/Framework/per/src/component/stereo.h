#ifndef HEIGHTMAP_PROJECTION__H
#define HEIGHTMAP_PROJECTION__H

#include <opencv2/opencv.hpp>
using namespace cv;

Mat getDisparity(Mat left_image, Mat right_image)
{
    cvtColor( left_image,left_image, CV_BGR2GRAY);
    cvtColor( right_image,right_image,CV_BGR2GRAY);
    IplImage temp=left_image;
    IplImage temp2=right_image;
    CvMat *matf= cvCreateMat ( temp.height, temp.width, CV_16S);
    CvStereoBMState * state=cvCreateStereoBMState(CV_STEREO_BM_NARROW,16*10);
    cvFindStereoCorrespondenceBM(&temp,&temp2,matf,state);
    CvMat * disp_left_visual= cvCreateMat(temp.height, temp.width, CV_8U);
    cvConvertScale( matf, disp_left_visual, -16 );
    cvNormalize( matf, matf, 0, 256, CV_MINMAX, NULL );
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
    for(int i = 0; i < img.rows; i++)
	   for(int j = 0; j < img.cols; j++)
	       if(img.at<uchar>(i, j) > 150) img.at<uchar>(i, j) = 0;
	       else img.at<uchar>(i, j) = (int)(img.at<uchar>(i, j)*3) ;
    return img;
}



#endif
