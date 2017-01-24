#ifndef _MISC_H_
#define _MISC_H_

#include <iostream>
#include <iomanip>
#include <cmath>  /* for std::abs(double) */
#include <stdexcept>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "mvg.h"

using std::string;
using cv::Mat;
using std::stringstream;
using cv::Point2f;
using cv::Vec3i;
using std::vector;

typedef vector<cv::KeyPoint> KeyPoints;
typedef Vec3i Match; //i1, i2, dist
typedef vector<Match> Matches;
typedef vector<Point2f> Points2f;
typedef Mat Descriptors;
typedef std::pair<Mat,Mat> image_pair;

bool isEqual(double x, double y);
bool isEqual(float x, float y);

template<typename T>
string _str(const Mat& m, bool include_dims=true, int truncate=16,
            int precision=2)
{
    stringstream ss;
    //ss << std::setprecision(precision) << fixed;
    if (include_dims)
    {
        ss << "(" << m.rows << "x" << m.cols << ") ";
    }
    ss << "[";
    for(int k=0, i=0; i<m.rows; ++i)
    {
        if (i>0)
            ss << " ";
        for(int j=0; j<m.cols; ++j, ++k)
        {
            ss << m.at<T>(i,j);
            if (j<m.cols-1)
                ss<<",";
            if (k==truncate)
            {
                ss << "...]";
                return ss.str();
            }
        }
        if (i<m.rows-1)
            ss << ";";
    }
    ss << "]";
    return ss.str();
}


template<class T>
Mat e2h(const Mat& X)
{
    Mat Xh(X.rows+1, X.cols, cv::DataType<T>::type);

    for(int i=0; i<X.rows; ++i)
    {
        for(int j=0; j<X.cols; ++j)
        {
            Xh.at<T>(i,j) = X.at<T>(i,j);
        }
    }
    for(int j=0; j<X.cols; ++j)
    {
        Xh.at<T>(Xh.rows-1,j) = 1.0;
    }
    return Xh;
}

template<class T>
Mat h2e(const Mat& X)
{
    Mat Xh(X.rows-1, X.cols, X.type());
    
    for(int i=0; i<Xh.rows; ++i)
    {
        for(int j=0; j<Xh.cols; ++j)
        {
            if (isEqual(abs(X.at<T>(X.rows-1,j)),.0f))
                throw std::overflow_error("divide by zero in h2e");
            Xh.at<T>(i,j) = X.at<T>(i,j)/X.at<T>(X.rows-1,j);
        }
    }
    return Xh;
}

#include <chrono>
template<typename TimeT = std::chrono::milliseconds>
struct measure
{
    template<typename F>
    static typename TimeT::rep execution(F const &func)
    {
        auto start = std::chrono::system_clock::now();
        func();
        auto duration = std::chrono::duration_cast< TimeT>(
            std::chrono::system_clock::now() - start);
        return duration.count();
    }
};

void randomsample(int n, int N, std::vector<int> & samples);
/** convert 6-dof motion parameter vector into the rigid transformation matrix 
 * \param tr input: must contain 6 motion params: [rx,ry,rz,tx,ty,tz]
 * \param T  must be preallocated 4x4
 */
void tr2mat(std::vector<double> tr,Mat T);
void save2features(const cv::Mat& im1, const cv::Mat& im2, Point2f p1,
		   vector<Point2f> neighbors, Point2f best_p2, const string& file_name,
		   int lim=INT_MAX);
void save2blend(const cv::Mat& im1, const cv::Mat& im2, const Mat& x,
		const string& file_name, int lim=INT_MAX);
void save2blend(const cv::Mat& im1, const cv::Mat& im2, const KeyPoints& kp1,
		const KeyPoints& kp2, const Matches &match, const string& file_name,
		int lim=INT_MAX);

bool save1reproj(const Mat& im, const Mat& X,const Mat& x, const Mat& P, const string& file_name);
void save4(const Mat& im1, const Mat& im1_prev,
	   const Mat& im2, const Mat& im2_prev,
	   const KeyPoints& kp1, const KeyPoints& kp1_prev,
	   const KeyPoints& kp2, const KeyPoints& kp2_prev,
	   const vector<cv::Vec4i>& ind, const string& file_name,
	   int lim=INT_MAX);
void save1(const Mat& im, const KeyPoints& kp, const string& file_name, int lim=INT_MAX,
	   cv::Scalar color=cv::Scalar(255,0,0), int thickness=1, int linetype=-1);

#endif /* _MISC_H_ */
