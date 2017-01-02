#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/flann/flann.hpp>

#include <map>
#include <stdexcept>
#include <vector>
#include <ctime>
#include <ctype.h>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <math.h>
#include <time.h>
#include <random>

#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <boost/log/trivial.hpp>
#include <fstream>

#include "viso.h"
#include "estimation.h"

using namespace std;
using namespace boost;

using cv::Vec3i;
using cv::Vec4i;
using cv::Vec2f;
using cv::DataType;
using cv::FM_RANSAC;
using cv::InputArray;
using cv::OutputArray;

int search_radius;

struct MatchParams
{
  bool enforce_epipolar;
  Mat F;
  double alg_thresh;
  double sampson_thresh;

  bool enforce_2nd_best;
  double ratio_2nd_best;

  bool allow_ann;
  int max_neighbors;
  double radius;

  MatchParams(Mat F) : enforce_epipolar(true),
                       sampson_thresh(1),
                       enforce_2nd_best(false),
                       ratio_2nd_best(.8),
                       allow_ann(true),
                       max_neighbors(200),
                       radius(300)
  {
    F.copyTo(this->F);
  };
  MatchParams() : enforce_epipolar(false), enforce_2nd_best(true),
                  ratio_2nd_best(.9), allow_ann(true), max_neighbors(250),
                  radius(200) {}
};

void
radiusSearch(cvflann::Index<cvflann::L1<float>>& index, Mat& query_points, 
             Mat& neighbors, float radius, Mat& p2, bool dbg=1)
{
  assert(query_points.type()==DataType<float>::type);
  assert(neighbors.type()==DataType<int>::type);
  cvflann::Matrix<float> dists(new float[neighbors.cols], 1, neighbors.cols);
  for(int i=0; i<query_points.rows; i++)
    {
      cvflann::Matrix<float> query(query_points.ptr<float>(i), 1, query_points.cols);
      cvflann::Matrix<int> nei(neighbors.ptr<int>(i), 1, neighbors.cols);
      int found = index.radiusSearch(query, nei, dists, radius, nei.cols);
      for(int j=found; j<nei.cols; ++j)
	{
	  nei.data[j] = -1;
	  dists.data[j] = -1;
	}
    }
}

/** Compute circles to prune outliers.
 * @param match_lr 3xN array of matches for (left,right) image pair
 * @param match_lr 3xN array of matches for previous (left,right) image pair 
 * @param match11 3xN array of matches for (left,prev left)
 * @param match22 3xN array of matches for (right,prev right)
 */
void
match_circle(const Matches& match_lr,
             const Matches& match_lr_prev,
             const Matches& match11,
             const Matches& match22,
             vector<Vec4i>& circ_match,
             Matches& match_pcl)
{
  // iterate over left features that have a match
  for(int i=0; i<match_lr.size(); ++i)
    {
      int ileft = match_lr.at(i)[0], iright = match_lr.at(i)[1];
      // go over all matches to left prev
      for(int j=0; j<match11.size(); ++j)
	{
	  if (match11.at(j)[0] == ileft)
	    {
	      int ileft_prev = match11.at(j)[1];
	      for(int k=0; k<match_lr_prev.size(); ++k)
		{
		  if (match_lr_prev.at(k)[0] == ileft_prev)
		    {
		      int iright_prev = match_lr_prev.at(k)[1];
		      for(int l=0; l<match22.size(); ++l)
			{
			  if (match22.at(l)[1] == iright_prev)
			    {
			      if (match22.at(l)[0] == iright)
				{
				  circ_match.push_back(Vec4i(ileft, iright, ileft_prev, iright_prev));
				  match_pcl.push_back(Match(i,k));
				}
			    }
			}
		    }
		}
	    }
	}
    }
}

// each keypoint (x,y) is a raw in the resut matrix
Mat
kp2mat(const KeyPoints& kp)
{
  Mat mat(kp.size(), 2, DataType<float>::type);
  for(int i=0; i<kp.size(); ++i)
    {
      mat.at<float>(i,0) = kp.at(i).pt.x;
      mat.at<float>(i,1) = kp.at(i).pt.y;
    }
  return mat;
}

double
algebricDistance(const Mat& F, const Point2f& p1, const Point2f& p2)
{
  assert(F.type() == DataType<double>::type);
  float
    a0=p1.x, a1=p1.y, a2=1,
    b0=p2.x, b1=p2.y, b2=1;
  return
    b0*F.at<double>(0,0)*a0 +
    b0*F.at<double>(0,1)*a1 +
    b0*F.at<double>(0,2)*a2 +
    b1*F.at<double>(1,0)*a0 +
    b1*F.at<double>(1,1)*a1 +
    b1*F.at<double>(1,2)*a2 +
    b2*F.at<double>(2,0)*a0 +
    b2*F.at<double>(2,1)*a1 +
    b2*F.at<double>(2,2)*a2 ;
}

/*
  distance = sum(p2'Fp1)/n
*/
double
algebricDistance(const Mat& F, const Points2f& p1, const Points2f& p2)
{
  assert(p1.size()==p2.size());
  double err=0;
  for(Points2f::const_iterator i1=p1.begin(), i2=p2.begin();
      i1 != p1.end();
      ++i1, ++i2)
    {
      err += algebricDistance(F, *i1, *i2);
    }
  return err/p1.size();
}

void
collect_matches(const KeyPoints& kp1, const KeyPoints &kp2,
		const Matches &match, Mat &x)
{
  x.create(4, match.size(), DataType<double>::type);
  for(int i=0; i<match.size(); ++i)
    {
      int i1 = match.at(i)[0], i2 = match.at(i)[1];
      x.at<double>(0,i) = kp1.at(i1).pt.x;
      x.at<double>(1,i) = kp1.at(i1).pt.y;
      x.at<double>(2,i) = kp2.at(i2).pt.x;
      x.at<double>(3,i) = kp2.at(i2).pt.y;
    }
}


/*
 * p2'Fp1 = 0
 */
double
sampsonDistance(const Mat& F,const Point2f& p1,const Point2f &p2)
{
  assert(F.type() == cv::DataType<double>::type);
  double 
    Fx0 = F.at<double>(0,0)*p1.x + F.at<double>(0,1)*p1.y + F.at<double>(0,2),
    Fx1 = F.at<double>(1,0)*p1.x + F.at<double>(1,1)*p1.y + F.at<double>(1,2),
    Ftx0= F.at<double>(0,0)*p2.x + F.at<double>(1,0)*p2.y + F.at<double>(2,0),
    Ftx1= F.at<double>(0,1)*p2.x + F.at<double>(1,1)*p2.y + F.at<double>(2,1);
  float ad = algebricDistance(F, p1, p2);
  return ad*ad/(Fx0*Fx0+Fx1*Fx1+Ftx0*Ftx0+Ftx1*Ftx1);
}

/* p2Fp1=0 */
void
match_desc(const KeyPoints& kp1, const KeyPoints& kp2,
           const Descriptors& d1, const Descriptors& d2,
           Matches &match, const MatchParams& sp,
           Mat i1, Mat i2, const boost::filesystem::path& dbg_dir, bool dbg=false)
{
  
  static int iter_num = 0;
  iter_num += 1;
  
  const clock_t begin_time = clock();
  match.clear();
  BOOST_ASSERT_MSG(d1.cols==d2.cols, (boost::format("d1.cols=%d,d2.cols=%d") % d1.cols % d2.cols).str().c_str());
  // cv::flann::Index index = (sp.allow_ann) ? 
  //     cv::flann::Index(kp2mat(kp2), cv::flann::KDTreeIndexParams(16), ::cvflann::FLANN_DIST_L1) :
  //     cv::flann::Index(kp2mat(kp2), cv::flann::LinearIndexParams(), ::cvflann::FLANN_DIST_L1);
  Mat kp1m = kp2mat(kp1), kp2m = kp2mat(kp2), neighbors(kp1m.rows, sp.max_neighbors,
                                                        DataType<int>::type, Scalar(-1));
  cvflann::Matrix<float> dataset((float*)kp2m.data, (size_t)kp2m.rows, (size_t)kp2m.cols);
  //cvflann::Index<cvflann::L1<float>> index(dataset, cvflann::KDTreeIndexParams(16));
  cvflann::Index<cvflann::L1<float>> index(dataset, cvflann::LinearIndexParams());
  radiusSearch(index, kp1m, neighbors, sp.radius, kp2m, !sp.enforce_epipolar);
  ofstream debug;
  if (dbg) debug.open((boost::format((dbg_dir/"feat%03d.txt").string())%iter_num).str());
  for(int i=0; i<kp1.size(); ++i)
    {
      Point2f p1 = kp1.at(i).pt;
      if (dbg)
	debug << "**" << i << ";" << p1.x << "," << p1.y << endl;
      string file_name = (boost::format((dbg_dir/"feat%03d_%03d.jpg").string()) % iter_num % i).str();
      vector<Point2f> neigh;
      for(int j=0, nind=neighbors.at<int>(i,j);
	  j<neighbors.cols && nind>0; ++j, nind=neighbors.at<int>(i,j))
	{
	  Point2f p2 = kp2.at(nind).pt;
	  neigh.push_back(p2);
	}
      if (!neigh.size())
	continue;
      double best_d1 = DBL_MAX, best_d2 = DBL_MAX;
      Point2f best_p2;
      pair<double,double> best_e;
      int best_idx = -1;
      for(int j=0, nind=neighbors.at<int>(i,j);
	  j<neighbors.cols && nind>0; ++j, nind=neighbors.at<int>(i,j))
	{
	  Point2f p2 = kp2.at(nind).pt;
	  if (sp.enforce_epipolar)
	    {
	      double sampson_dist = sampsonDistance(sp.F, p1, p2);
	      if (!std::isfinite(sampson_dist) || sampson_dist > sp.sampson_thresh)
		continue;
	    }
	  double d = cv::norm(d2.row(nind)-d1.row(i), cv::NORM_L1);
	  if (dbg) debug << d << ";" << p2.x << "," << p2.y << endl;
	  if (d <= best_d1)
	    {
	      best_d2 = best_d1;
	      best_d1 = d;
	      best_idx = nind;
	      best_p2 = kp2.at(nind).pt;
	    } else if (d <= best_d2)
	    best_d2 = d;
	}
      if (best_idx >= 0)
	{
	  if (sp.enforce_2nd_best)
	    {
	      if (best_d1 < best_d2*sp.ratio_2nd_best)
		{
		  match.push_back(Match(i, best_idx, best_d1));
		  //          printf("%g ",best_d1);
		}
	    } else {
	    match.push_back(Match(i, best_idx, best_d1));
	    //        printf("%g ",best_d1);
	    if (dbg) {
	      debug << "-" << best_d1 << ";" << best_p2.x << "," << best_p2.y << endl;
	      save2features(i1,i2,p1,neigh,best_p2,file_name);
	    }
	  }
	}
    }
  if (dbg) debug.close();
  std::sort(match.begin(), match.end(), [](const Match &a, const Match &b) { return a[2]<b[2];});
  //BOOST_LOG_TRIVIAL(info) << "match time [s]:" << float(clock()-begin_time)/CLOCKS_PER_SEC;
}

void
radiusSearch(cv::flann::Index& index, Mat& points, Mat& neighbors, float radius, Mat& p2)
{
  assert(points.type()==DataType<float>::type);
  assert(neighbors.type()==DataType<int>::type);
  for(int i=0; i<points.rows; i++)
    {
      Mat p(1, points.cols, DataType<float>::type, points.ptr<float>(i)),
        n(1, neighbors.cols, CV_32SC1, neighbors.ptr<int>(i)),
        dist(1, neighbors.cols, DataType<float>::type);
      int found = index.radiusSearch(p, n, dist, radius, neighbors.cols);
      cout << "found=" << found << endl;
      cout << "p=" << p << endl;
      cout << "n=" << n << endl;
      cout << "dist=" << dist << endl;
      for(int j=0; j<neighbors.cols && j<found; ++j)
	{
	  int ind = n.at<int>(0,j);
	  assert(ind>=0);
	  Mat pt2 = p2.row(ind);
	  double dst = abs(p.at<float>(0,0)-pt2.at<float>(0,0))+abs(p.at<float>(0,1)-pt2.at<float>(0,1));
	  cout << "dst=" << dst << ",";
	  cout << "neighbor=[" << pt2.at<float>(0,0) << "," << pt2.at<float>(0,1) << ";" << endl;
	  if (0 && dst>radius)
	    {
	      cout << "radius violation: query=" << p 
		   << "; neighbor=" << pt2
		   << "; L1=" << dst
		   << "; dist=" << dist.at<float>(0,j) << endl;
	      cout << "j=" << j << endl;
	      cout << "ind=" << ind << endl;
	      cout << "found=" << found << endl;
	      cout << "n=" << n << endl;
	      cout << "dist=" << dist << endl;
	    }
	}
      cout << endl;
    }
}


class HarrisBinnedFeatureDetector : public cv::FeatureDetector
{
public:
  // descriptor radius is used only to init KeyPoints
  HarrisBinnedFeatureDetector(int radius, int n, int nbinx=24, int nbiny=5,
			      float k = .04, int block_size=3, int aperture_size=5)
    : m_radius(radius), m_nbinx(nbinx), m_nbiny(nbiny), 
      m_block_size(block_size), m_aperture_size(aperture_size), m_n(n)
  {
    assert(nbinx>0 && nbiny>0);
  }
  
protected:
  
  void
  detectImpl(const Mat& image, KeyPoints& kp, const Mat& mask=Mat()) const
  {
    Mat harris_response;
    // M_c = det(A) - k*trace^2(A), the range for k \in [0.04, 0.15]
    cv::cornerHarris(image, harris_response, m_block_size, m_aperture_size,
		     m_k, cv::BORDER_DEFAULT);
    assert(harris_response.type() == DataType<float>::type);
    Mat maxima, msk = Mat::ones(harris_response.rows, harris_response.cols, CV_8U);
    nonMaximaSuppression(harris_response, 3, maxima, msk);
    harris_response.setTo(0, maxima==0);
    int stridex = (int)image.cols/m_nbinx, stridey = (int)image.rows/m_nbiny;
    assert(stridex>0 && stridey>0);
    struct elem
    {
      int x, y;
      float val;
      elem(int x, int y, float val) : x(x), y(y), val(val) {}
      elem() : x(-1), y(-1), val(NAN) {}
      bool operator<(const elem& other) const{ return val<other.val; }
    };
    int corners_per_block = (int)m_n/(m_nbinx*m_nbiny);
    vector<elem> v;
    v.reserve(stridex*stridey);
    for(int binx=0; binx<m_nbinx; ++binx)
      {
	for(int biny=0; biny<m_nbiny; ++biny)
	  {
	    for(int x=binx*stridex; x<(binx+1)*stridex && x<harris_response.cols; ++x)
	      {
		for(int y=biny*stridey; y<(biny+1)*stridey && y<harris_response.rows; ++y)
		  {
		    float response = abs(harris_response.at<float>(y,x));
		    if (response < 1e-8) continue;
		    v.push_back(elem(x,y,response));
		  }
	      }
	    int m = (v.size()>corners_per_block) ? v.size()-corners_per_block : 0;
	    if (m>0)
	      std::nth_element(v.begin(), v.begin()+m, v.end());
	    for(vector<elem>::iterator iter=v.begin()+m; iter<v.end(); ++iter)
	      {
		KeyPoint keypoint;
		keypoint.pt = Point2f(iter->x, iter->y);
		keypoint.response = iter->val;
		keypoint.size = 2*m_radius+1;
		if (iter->x > m_radius && iter->y > m_radius &&
		    iter->x < harris_response.cols-m_radius &&
		    iter->y < harris_response.rows-m_radius)
		  kp.push_back(keypoint);
	      }
	    v.clear();
	  }
      }
    std::cout << "; " << kp.size() << " corners";
  }
  int m_radius, m_nbinx, m_nbiny, m_block_size, m_aperture_size,m_n;
  float m_k;
};

class PatchExtractor : public cv::DescriptorExtractor
{
public:
  PatchExtractor(int descriptor_radius) 
    : m_descriptor_radius(descriptor_radius) {}
protected:
  int m_descriptor_radius;
  
  int defaultNorm() const
  {
    return cv::NORM_L1;
  }
  int descriptorType() const
  {
    return DataType<float>::type;
  }
  
  int 
  descriptorSize() const
  {
    return (2*m_descriptor_radius+1)*(2*m_descriptor_radius+1);
  }
  
  void computeImpl(const Mat& image, std::vector<KeyPoint>& kp, Mat& d) const
  {
    Size sz = image.size();
    int cols = sz.width, rows = sz.height;
    Mat sob(rows, cols, DataType<int>::type, Scalar(0));
    Mat(kp.size(), descriptorSize(), DataType<float>::type, Scalar(0)).copyTo(d);
    assert(d.data);
    assert(image.type() == cv::DataType<unsigned char>::type);
    //Sobel(image, sob, sob.type(), 1, 0, 3, 1, 0, cv::BORDER_REFLECT_101);
    for(int k=0; k<kp.size(); ++k)
      {
	Point2i p = kp.at(k).pt;
	for(int i=-m_descriptor_radius,col=0; i<=m_descriptor_radius; i+=1)
	  {
	    for(int j=-m_descriptor_radius; j<=m_descriptor_radius; j+=1,++col)
	      {
		float val = (p.y+i>0 && p.y+i<rows && p.x+j>0 && p.x+j<cols) ?
		  image.at<unsigned char>(p.y+i, p.x+j) : 0;
		//              sob.at<float>(p.y+i, p.x+j) : 0;
		d.at<float>(k,col) = val;
	      }
	  }
      }
  }
};

typedef Mat Descriptor;

void
localMatch(const KeyPoints& kp1, const KeyPoints& kp2,
           const Descriptor& d1, const Descriptor& d2,
           Matches& match, double thresh)
{
  double min_d = DBL_MAX;
  Match best_match;
  for(int i=0; i<kp1.size(); ++i)
    {
      for(int j=0; j<kp2.size(); ++j)
	{
	  double d = norm(kp1.at(i).pt-kp2.at(j).pt);
	  if (d>thresh)
	    continue;
	  d = cv::norm(d1.col(i)-d2.col(j));
	  if (d<min_d)
	    {
	      best_match = Match(i,j);
	    }
	}
      match.push_back(best_match);
    }
}

namespace viso_isl {

  cv::Mat
  StereoVO::getMotion()
  {
    Mat motion(4, 4, DataType<double>::type);
    tr2mat(prev.tr, motion);
    return motion;
  }
  
  bool
  StereoVO::process_stereo_frame(cv::Mat left, cv::Mat right)
  {
    static bool first = true;

    bool status = true;
  
    StereoImageGenerator::result_type stereo_pair;
  
    Mat d1, d2, X;
    KeyPoints kp1, kp2;
    Matches match_lr;
 
    assert(left.data && right.data);

    HarrisBinnedFeatureDetector detector(5, 1200 /*MAX_FEATURE_NUM*/);
    detector.detect(left, kp1);
    detector.detect(right, kp2);

    cout << " using " << kp1.size() << " keypoints in the 1st image" << endl;
    cout << " using " << kp2.size() << " keypoints in the 2nd image" << endl;
  
    PatchExtractor extractor(5);
    extractor.compute(left, kp1, d1);
    extractor.compute(right, kp2, d2);

    match_desc(kp1, kp2, d1, d2, match_lr, MatchParams(F), left, right, dbg_dir);
    Mat x;
    collect_matches(kp1, kp2, match_lr, x);
    X = triangulate_rectified<double>(x, param.calib.f, param.base, param.calib.cu,
				      param.calib.cv);
    
    if (first) {
      first = false;
      return true;
    }
        
    /* match left vs. left previous */
    Matches match11;
    MatchParams mp = MatchParams();
    mp.enforce_2nd_best = false;
    mp.radius = search_radius;
    match_desc(kp1, prev.kp1, d1, prev.d1, match11, mp, left, right, dbg_dir);

    /* match right vs. right prev */
    Matches match22;
    match_desc(kp2, prev.kp2, d2, prev.d2, match22, mp, left, right, dbg_dir);
  
    Matches match_pcl; 
    vector<Vec4i> circ_match;
    match_circle(match_lr, prev.match, match11, match22, circ_match, match_pcl);
    if (circ_match.size() < 3) {
      cout << "not enough matches in current circle: " << circ_match.size();
      return false;
    }
    std::cout << "number of circles: " << match_pcl.size() << endl;

    /* collect the points that participate in circular match */
    Mat Xp_c(3,circ_match.size(),DataType<double>::type),
      x_c(4,circ_match.size(),DataType<double>::type);
    for(int i=0;i<match_pcl.size();++i) {
      /* observed points in both images */
      x_c.at<double>(0,i) = x.at<double>(0,match_pcl[i][0]);
      x_c.at<double>(1,i) = x.at<double>(1,match_pcl[i][0]);
      x_c.at<double>(2,i) = x.at<double>(2,match_pcl[i][0]);
      x_c.at<double>(3,i) = x.at<double>(3,match_pcl[i][0]);
      /* previous 3d point */
      Xp_c.at<double>(0,i) = prev.X.at<double>(0,match_pcl[i][1]);
      Xp_c.at<double>(1,i) = prev.X.at<double>(1,match_pcl[i][1]);
      Xp_c.at<double>(2,i) = prev.X.at<double>(2,match_pcl[i][1]);
    }

    vector<int> inliers;
    vector<double> tr = prev.tr;
    if (ransac_minimize_reproj(Xp_c, x_c, tr, inliers, param)) {
      // convert motion vector into the transformation matrix
      Mat tr_mat(4,4,DataType<double>::type);
      tr2mat(tr, tr_mat);

      // prev pose, s.t. X_0 = pose*X_{i-1}
      Mat pose = poses.back();

      // pose composition, now X_0 = pose*X_{i}
      pose = pose*tr_mat.inv();

      // store the pose
      poses.push_back(pose.clone());

      Mat x_inl(4, inliers.size(), cv::DataType<double>::type);
      for(int i=0; i<inliers.size(); ++i) {
	x_inl.at<double>(0,i) = x.at<double>(0, inliers[i]);
	x_inl.at<double>(1,i) = x.at<double>(1, inliers[i]);
	x_inl.at<double>(2,i) = x.at<double>(2, inliers[i]);
	x_inl.at<double>(3,i) = x.at<double>(3, inliers[i]);
      }
      prev.tr = tr;
    } else {
      std::cout << "; **error**: failed to solve rigid motion";
      // I put in an identity in order not to break the correspondence vs. the gt
      poses.push_back(Mat::eye(4,4,DataType<double>::type));
      prev.tr = vector<double>(6,0);
      status = false;
    }

    // save cur frame stuff
    left.copyTo(prev.left);
    right.copyTo(prev.right);
    d1.copyTo(prev.d1);
    d2.copyTo(prev.d2);
    prev.kp1 = kp1;
    prev.kp2 = kp2;
    prev.match = match_lr;
    X.copyTo(prev.X);
    return status;
  }


void
compute_J(const Mat& X, const Mat& observe, vector<double> &tr, const struct params &param,
          const vector<int> &active, Mat& J, Mat& predict, Mat& residual)
{
  // extract motion parameters
  double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
  double tx = tr[3]; double ty = tr[4]; double tz = tr[5];
    
  // precompute sine/cosine
  double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);
  double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);
    
  // compute rotation matrix and derivatives
  double r00    = +cy*cz;          double r01    = -cy*sz;          double r02    = +sy;
  double r10    = +sx*sy*cz+cx*sz; double r11    = -sx*sy*sz+cx*cz; double r12    = -sx*cy;
  double r20    = -cx*sy*cz+sx*sz; double r21    = +cx*sy*sz+sx*cz; double r22    = +cx*cy;
  double rdrx10 = +cx*sy*cz-sx*sz; double rdrx11 = -cx*sy*sz-sx*cz; double rdrx12 = -cx*cy;
  double rdrx20 = +sx*sy*cz+cx*sz; double rdrx21 = -sx*sy*sz+cx*cz; double rdrx22 = -sx*cy;
  double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
  double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
  double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
  double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
  double rdrz10 = -sx*sy*sz+cx*cz; double rdrz11 = -sx*sy*cz-cx*sz;
  double rdrz20 = +cx*sy*sz+sx*cz; double rdrz21 = +cx*sy*cz-sx*sz;
    
  // loop variables
  double X1p,Y1p,Z1p;
  double X1c,Y1c,Z1c,X2c;
  double X1cd,Y1cd,Z1cd;
  //    printf("sample: %d,%d,%d\n",active[0],active[1],active[2]);
  // for all observations do
  for (int i=0; i<active.size(); i++)
    {
      // get 3d point in previous coordinate system
      X1p = X.at<double>(0,active[i]);
      Y1p = X.at<double>(1,active[i]);
      Z1p = X.at<double>(2,active[i]);
      //        cout << "X1p=" << X1p << ",Y1p=" << Y1p << ",Z1p=" << Z1p << endl;

      // compute 3d point in current left coordinate system
      X1c = r00*X1p+r01*Y1p+r02*Z1p+tx;
      Y1c = r10*X1p+r11*Y1p+r12*Z1p+ty;
      Z1c = r20*X1p+r21*Y1p+r22*Z1p+tz;
      //        cout << "X1c=" << X1c << ",Y1p=" << Y1c << ",Z1c=" << Z1c << endl;
        
      // weighting
      double weight = 1.0;
      if (true)
	weight = 1.0/(fabs(observe.at<double>(0,i)-param.calib.cu)/fabs(param.calib.cu) + 0.05);
        
      // compute 3d point in current right coordinate system
      X2c = X1c-param.base;
      //        cout << "X2c:" << X2c << endl;
      // for all paramters do
      for (int j=0; j<6; j++)
	{
	  // derivatives of 3d pt. in curr. left coordinates wrt. param j
	  switch (j)
	    {
	    case 0: X1cd = 0;
	      Y1cd = rdrx10*X1p+rdrx11*Y1p+rdrx12*Z1p;
	      Z1cd = rdrx20*X1p+rdrx21*Y1p+rdrx22*Z1p;
	      break;
	    case 1: X1cd = rdry00*X1p+rdry01*Y1p+rdry02*Z1p;
	      Y1cd = rdry10*X1p+rdry11*Y1p+rdry12*Z1p;
	      Z1cd = rdry20*X1p+rdry21*Y1p+rdry22*Z1p;
	      break;
	    case 2: X1cd = rdrz00*X1p+rdrz01*Y1p;
	      Y1cd = rdrz10*X1p+rdrz11*Y1p;
	      Z1cd = rdrz20*X1p+rdrz21*Y1p;
	      break;
	    case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
	    case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
	    case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
	    }
            
	  // set jacobian entries (project via K)
	  J.at<double>(4*i+0,j) = weight*param.calib.f*(X1cd*Z1c-X1c*Z1cd)/(Z1c*Z1c); // left u'
	  J.at<double>(4*i+1,j) = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // left v'
	  J.at<double>(4*i+2,j) = weight*param.calib.f*(X1cd*Z1c-X2c*Z1cd)/(Z1c*Z1c); // right u'
	  J.at<double>(4*i+3,j) = weight*param.calib.f*(Y1cd*Z1c-Y1c*Z1cd)/(Z1c*Z1c); // right v'
	  //            printf("observation %d, param %d: %g,%g,%g,%g\n",i,j,J.at<double>(4*i+0,j),J.at<double>(4*i+1,j),J.at<double>(4*i+2,j),J.at<double>(4*i+3,j));
	}
        
      // set prediction (project via K)
      predict.at<double>(0,i) = param.calib.f*X1c/Z1c+param.calib.cu; // left u
      predict.at<double>(1,i) = param.calib.f*Y1c/Z1c+param.calib.cv; // left v
      predict.at<double>(2,i) = param.calib.f*X2c/Z1c+param.calib.cu; // right u
      predict.at<double>(3,i) = param.calib.f*Y1c/Z1c+param.calib.cv; // right v
    
      // set residuals
      residual.at<double>(4*i+0,0) = weight*(observe.at<double>(0,active[i])-predict.at<double>(0,i));
      residual.at<double>(4*i+1,0) = weight*(observe.at<double>(1,active[i])-predict.at<double>(1,i));
      residual.at<double>(4*i+2,0) = weight*(observe.at<double>(2,active[i])-predict.at<double>(2,i));
      residual.at<double>(4*i+3,0) = weight*(observe.at<double>(3,active[i])-predict.at<double>(3,i));
    }
}

template<typename T>
ostream& operator<< (ostream& out, const vector<T> v) {
  int last = v.size() - 1;
  out << "[";
  for(int i = 0; i < last; i++)
    out << v[i] << ", ";
  out << v[last] << "]";
  return out;
}
/* calculate the support set for current ransac model */
std::pair<vector<int>,double>
get_inliers(const Mat& X, const Mat& observe, vector<double> &tr, const struct params& param)
{
  vector<int> active;
  for (int i=0; i<X.cols; ++i)
    active.push_back(i);

  int num_pts = active.size();
  Mat J(4*num_pts,6,DataType<double>::type), /* all of these will not be used */
    residual(4*num_pts,1,DataType<double>::type),
    predict(4,num_pts,DataType<double>::type);
  compute_J(X,observe,tr,param,active,J,predict,residual);

  /* compute the inliers */
  vector<int> inliers;
  double err2 = 0, total_err2 = 0;
  for (int i=0; i<X.cols; ++i)
    {
      err2 = 
        pow(observe.at<double>(0,i)-predict.at<double>(0,i),2) +
        pow(observe.at<double>(1,i)-predict.at<double>(1,i),2) +
        pow(observe.at<double>(2,i)-predict.at<double>(2,i),2) +
        pow(observe.at<double>(3,i)-predict.at<double>(3,i),2);
      if (err2 < param.inlier_threshold*param.inlier_threshold)
	{
	  inliers.push_back(i);
	  total_err2 += err2;
	}
    }
  double rms = (inliers.size()) ? sqrt(total_err2/inliers.size()) :
    std::numeric_limits<double>::quiet_NaN();
  return std::make_pair(inliers,rms);
}

/** RANSAC wrapper for model estimation.
 * \param X 3d points as seen in previous stereo view (3xN)
 * \param observe pixel coordinates of the same features as \a X as seen in
 *        current view (4xN)
 * \param best_tr output: estimated 6-dof motion parameters vector. \a best_tr
 *        transforms from frame {i-1} to frame {i}, e.g., X_{i+1} = T(tr)*X_i
 * \param best_inliers \a best_tr support set
 */
bool
ransac_minimize_reproj(const Mat& X, 
                       const Mat& observe, 
                       vector<double>& best_tr,
                       vector<int>& best_inliers,
                       const struct viso_isl::params& param)
{
  int model_size = 3; /*!< min sample size to estimate 6-dof motion */
  vector<int> sample(model_size,0); /*!< current sample */
  vector<double> tr(6,0); /*!< current parameter vector */
    
  best_inliers.clear();
  for(int i=0; i<param.ransac_iter; ++i)
    {
      /* start search from 0 vector, this is rather arbitrary */
      std::fill(tr.begin(),tr.end(),0); 

      /* select sample */
      randomsample(3,X.cols,sample); 

      /* estimate model */
      if (minimize_reproj(X,observe,tr,param,sample) == false) 
	{
	  continue;
	} else {
	std::pair<vector<int>,double> p = get_inliers(X,observe,tr,param); /* get the support set for this model */
	if (p.first.size()>best_inliers.size()) /* save it as best */
	  {
	    best_inliers = p.first;
	    best_tr = tr;
	  }
      }
    }
  if (best_inliers.size()<6 ||
      minimize_reproj(X,observe,best_tr,param,best_inliers) == false)
    return false;

  std::pair<vector<int>,double> p = get_inliers(X,observe,best_tr,param);
  best_inliers = p.first;
  std::cout << "; best support set size:" << best_inliers.size()
            << ", reprojection error RMS: " << p.second;
  return true;
}

/** Gauss Newton minimization of the reprojection error.
 * \param X input set of 3d points as seen in previous view
 * \param observe pixel coordinates of the image points that correspond to the 3d
 *       points in X as seen in the current view
 * \param tr output argument, estimated parameter vector
 * \param param parameters?
 * \param active current sample
 */
bool
minimize_reproj(const Mat& X,
		const Mat& observe,
		vector<double>& tr,
                const struct params& param,
		const vector<int>& active)
{
  int num_pts = active.size();
  Mat A(6,6,DataType<double>::type), B(6,1,DataType<double>::type),
    J(4*num_pts,6,DataType<double>::type),
    residual(4*num_pts,1,DataType<double>::type),
    predict(4,num_pts,DataType<double>::type);
  double step_size = 1.0f;
  for(int i=0; i<100; ++i)
    {
      compute_J(X,observe,tr,param,active,J,predict,residual);
      Mat JtJ(6,6,DataType<double>::type), p_gn(6,1,DataType<double>::type); 

      mulTransposed(J,JtJ,true);
      if (solve(JtJ,J.t()*residual,p_gn,cv::DECOMP_LU) == false)
	return false;

      bool converged = true;
      for(int j=0;j<6;++j)
	{
	  if (fabs(p_gn.at<double>(j,0) > param.thresh))
	    {
	      converged = false;
	      break;
	    }
	}
      if (converged)
	return converged;
      for (int j=0;j<6;++j)
	tr[j] = tr[j] + step_size*p_gn.at<double>(j,0);
    }
  return false;
}
} //namespace viso_isl
