#ifndef _VISO_H
#define _VISO_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/imgproc/types_c.h>
//#include <opencv2/nonfree/nonfree.hpp>

#include <map>
#include <stdexcept>
#include <vector>
#include <iostream>
#include <iomanip>

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>

#include <boost/format.hpp>
#include <boost/filesystem.hpp>

#include <Eigen/Dense>

#include "mvg.h"
#include "misc.h"
#include "nms.h"

using namespace std;
using namespace boost;

using cv::Mat;
using cv::KeyPoint;
using cv::Vec2i;
using cv::FeatureDetector;
using cv::DescriptorExtractor;
using cv::Scalar;
using cv::FileStorage;
using cv::Vec6f;
using cv::Point2i;
using cv::waitKey;
using cv::Size;
using cv::Point;
using cv::Point2f;
using cv::Vec3i;

namespace viso_isl {

  class params {
  public:
  params()
    : ransac_iter(1000), inlier_threshold(3), save_debug(false), thresh(1e-6) {}
   
    int    ransac_iter;
    double inlier_threshold;
    double thresh;    // gradient norm threshold //
    bool   save_debug;
    double base;      // stereo pair baseline + camera intrinisics
    struct {
      double f;
      double cu;
      double cv;
    } calib;
  }; // class param
  
  class StereoVO {
  public:    
  StereoVO(const params& p) :dbg_dir("/tmp") {
      // fundamental for the rectified stereo pair
      F = cv::Mat(cv::Mat::zeros(3,3,cv::DataType<double>::type));
      F.at<double>(1,2) = -1;
      F.at<double>(2,1) =  1;
      
      poses.push_back(cv::Mat::eye(4,4,cv::DataType<double>::type));
    }
    cv::Mat getMotion();
    bool process_stereo_frame(const cv::Mat left, const cv::Mat right);
  private:
    params param;
    vector<Mat> poses;
    cv::Mat F;
    std::string dbg_dir;
    
    struct FrameData {
    FrameData() : tr(6,0) {}
      cv::Mat left, right, X, d1, d2;
      KeyPoints kp1, kp2;
      Matches match;
      vector<double> tr;  
    } prev;

  };
  bool
    ransac_minimize_reproj(const Mat& X,const Mat& observe,vector<double>& best_tr,vector<int>& best_inliers, const struct params& param);

  bool
    minimize_reproj(const Mat& X, const Mat& observe, vector<double>& tr, const struct params& param,
		    const vector<int>& active);

  class StereoImageGenerator
  {
  public:
    typedef boost::optional<image_pair> result_type;
    typedef pair<string, string> string_pair;
  StereoImageGenerator(const string_pair &mask,int begin=0, int end=INT_MAX)
    : m_mask(mask), m_index(begin), m_end(end) {}
    result_type operator()() {
      if (m_index > m_end)
	return result_type();
      string name0 = str(boost::format(m_mask.first) % m_index),
	name1 = str(boost::format(m_mask.second) % m_index);
      //	BOOST_LOG_TRIVIAL(debug) << "left image: " << name0;
      //BOOST_LOG_TRIVIAL(debug) << "right image: " << name1;

      image_pair pair = make_pair(cv::imread(name0, CV_LOAD_IMAGE_GRAYSCALE),
				  cv::imread(name1, CV_LOAD_IMAGE_GRAYSCALE));
      m_index++;
      return pair.first.data && pair.second.data ? result_type(pair) : result_type();
    }
    int current() const { return m_index; }
  private:
    int m_index, m_end;
    string_pair m_mask;
  };

  class MonoImageGenerator {
  public:
    typedef boost::optional<Mat> result_type;
  MonoImageGenerator(const string& mask,int begin=0, int end=INT_MAX)
    : m_mask(mask), m_index(begin), m_end(end) {}
    result_type operator()() {
      if (m_index > m_end)
	return result_type();
      string name = str(boost::format(m_mask) % m_index);
      Mat image = cv::imread(name, CV_LOAD_IMAGE_GRAYSCALE);
      m_index++;
      return image.data ? result_type(image) : result_type();
    }
  private:
    int m_index, m_end;
    string m_mask;
  };

  /*** Read the intrinsic and extrinsic parameters
       Note: this works for KITTI, will probably need to be updated for another dataset 
  */
  void
    readCameraParams(const string &intrinsics_name, const string &extrinsics_name,
		     StereoCam &p);

  Mat
    getFundamentalMat(const Mat& R1, const Mat& t1,
		      const Mat& R2, const Mat& t2,
		      const Mat& cameraMatrix);
  void
    findConstrainedCorrespondences(const Mat& F, const KeyPoints& kp1,
				   const KeyPoints& kp2,const Mat& d1,
				   const Mat& d2, Matches& matches,
				   double eps, double ratio);

  vector<Mat>
    sequence_odometry(const Mat& p1, const Mat& p2, StereoImageGenerator& images, const boost::filesystem::path& dbg_dir);


  void
    match_l2_2nd_best(const Descriptors& d1, const Descriptors& d2,
		      Matches& match, float ratio=0.7);

  /* p2Fp1=0 */
  void
    match_epip_constraint(const cv::Mat& F, const KeyPoints& kp1, 
			  const KeyPoints& kp2, const Descriptors& d1,
			  const Descriptors& d2, Matches &match,
			  double ratio, double samp_thresh, double alg_thresh);
  void
    save2(const cv::Mat& m1, const cv::Mat& m2, const KeyPoints& kp1,
	  const KeyPoints& kp2, const Matches &match, const string &file_name,
	  int lim=50);
  void
    calibratedSFM(const Mat& K, MonoImageGenerator& images);
}; // namespace viso_isl

#endif