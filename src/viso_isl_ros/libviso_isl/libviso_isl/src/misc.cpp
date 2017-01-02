#include "misc.h"

cv::Scalar red = cv::Scalar(0,0,255), green = cv::Scalar(0,255,0), blue = cv::Scalar(255,0,0);

bool isEqual(double x, double y)
{
    const double epsilon = 1e-6;
    return std::abs(x - y) <= epsilon * std::abs(x);
  // see Knuth section 4.2.2 pages 217-218
}

bool isEqual(float x, float y)
{
    const float epsilon = 1e-6;
    return std::abs(x - y) <= epsilon * std::abs(x);
  // see Knuth section 4.2.2 pages 217-218
}


// John D. Cook, http://stackoverflow.com/a/311716/15485
void
randomsample(int n, int N, std::vector<int> & samples)
{
  int t = 0; // total input records dealt with
  int m = 0; // number of items selected so far
  double u;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> dis(0, 1);
  while (m < n)
  {
    u = dis(gen);

    if ((N - t)*u >= n - m) {
      t++;
    } else {
      samples[m] = t;
      t++; m++;
    }
  }
}

void
tr2mat(std::vector<double> tr, Mat T)
{
  // extract parameters
  double rx = tr[0];
  double ry = tr[1];
  double rz = tr[2];
  double tx = tr[3];
  double ty = tr[4];
  double tz = tr[5];
    
  // precompute sine/cosine
  double sx = sin(rx);
  double cx = cos(rx);
  double sy = sin(ry);
  double cy = cos(ry);
  double sz = sin(rz);
  double cz = cos(rz);
    
  // compute transformation
  T.at<double>(0,0) = +cy*cz;
  T.at<double>(0,1) = -cy*sz;
  T.at<double>(0,2) = +sy;
  T.at<double>(0,3) = tx;

  T.at<double>(1,0) = +sx*sy*cz+cx*sz;
  T.at<double>(1,1) = -sx*sy*sz+cx*cz;
  T.at<double>(1,2) = -sx*cy;
  T.at<double>(1,3) = ty;

  T.at<double>(2,0) = -cx*sy*cz+sx*sz;
  T.at<double>(2,1) = +cx*sy*sz+sx*cz;
  T.at<double>(2,2) = +cx*cy;
  T.at<double>(2,3) = tz;
  
  T.at<double>(3,0) = 0;
  T.at<double>(3,1) = 0;
  T.at<double>(3,2) = 0;
  T.at<double>(3,3) = 1;
}

void
drawPoints(Mat& im, const Mat& x, const cv::Scalar& color, int thickness, int linetype)
{
  assert(x.type() == cv::DataType<double>::type);
  for(int i=0; i<x.cols; ++i)
    circle(im, cv::Point(x.at<double>(0,i), x.at<double>(1,i)), thickness, color, linetype);
}

void
drawPoints(Mat& im, const KeyPoints& kp, int lim, const cv::Scalar& color,
           int thickness, int linetype)
{
  for(int i=0; i<kp.size() && i<lim; ++i)
    circle(im, kp.at(i).pt, thickness, color, linetype);
}

void
save1(const Mat& im, const KeyPoints& kp, const string& file_name, int lim,
      cv::Scalar color, int thickness, int linetype)
{
  Mat im_rgb;
  cv::cvtColor(im, im_rgb, CV_GRAY2RGB);
  drawPoints(im_rgb, kp, lim, color, thickness, linetype);
  cv::imwrite(file_name, im_rgb);
}

void
save2reproj(const Mat& im1, const Mat& im2, const Mat& X, const Mat& P1,
	    const Mat& P2, const string& file_name)
{
  Mat blend, im;
  addWeighted(im1, .5, im2, .5, 0.0, blend);
  cv::cvtColor(blend, im, CV_GRAY2RGB);
  Mat x1 = projectPoints(X,P1),
      x2 = projectPoints(X,P2);
  int thickness = 1, linetype = 1;
  for(int i=0; i<X.cols; ++i) {
    cv::Point
        p1 = cv::Point(x1.at<double>(0,i), x1.at<double>(1,i)),
        p2 = cv::Point(x1.at<double>(0,i), x1.at<double>(1,i));
    circle(im, p1, thickness, red, linetype);
    circle(im, p2, thickness, green, linetype);
    line(im, p1, p2, blue);
  }
  cv::imwrite(file_name, im);
}

bool
save1reproj(const Mat& im, const Mat& X,const Mat& x, const Mat& P, const string& file_name)
{
  cv::Mat im_rgb;
  cv::cvtColor(im, im_rgb, CV_GRAY2RGB);
  drawPoints(im_rgb, x, cv::Scalar(255,0,0), 1, -1);
  drawPoints(im_rgb, projectPoints(X, P), cv::Scalar(0,255,0), 3, 1);
  cv::imwrite(file_name, im_rgb);
}


/* concatenate a pair of matrices vertically
   m1, m2 must have the same number of columns
*/
void
save2(const cv::Mat& im1, const cv::Mat& im2, const KeyPoints& kp1,
      const KeyPoints& kp2, const Matches &match, const string& file_name,
      int lim)
{
  cv::Mat im_t = vcat<uchar>(im1, im2), im;
  cv::cvtColor(im_t, im, CV_GRAY2RGB);
  for (int i=0; i<kp1.size(); ++i)
  {
    circle(im, kp1.at(i).pt, 2, cv::Scalar(255,0,0), -1);
  }
  for (int i=0; i<kp2.size(); ++i)
  {
    circle(im, cv::Point(kp2.at(i).pt.x, kp2.at(i).pt.y+im1.rows), 2, cv::Scalar(255,0,0), -1);
  }
  for(int i=0; i<match.size() && i<lim; ++i)
  {
    cv::Point 
        p1 = kp1[match.at(i)[0]].pt,
        p2 = kp2[match.at(i)[1]].pt;
    p2.y += im1.rows;
    line(im, p1, p2, cv::Scalar(255));
  }
  cv::imwrite(file_name, im);
}

void
save2blend(const cv::Mat& im1, const cv::Mat& im2, const KeyPoints& kp1,
           const KeyPoints& kp2, const Matches &match, const string& file_name,
           int lim)
{
  cv::Mat blend, im;
  addWeighted(im1, .5, im2, .5, 0.0, blend);
  cv::cvtColor(blend, im, CV_GRAY2RGB);
  for(int i=0;i<match.size() && i<lim; ++i)
  {
    cv::Point
        p1 = kp1[match.at(i)[0]].pt,
        p2 = kp2[match.at(i)[1]].pt;
    circle(im, p1, 1, cv::Scalar(0,255,0), -1);
    line(im, p1, p2, cv::Scalar(255,0,0));
  }
  cv::imwrite(file_name, im);
}

void
save_keypoints(const Mat& x, const string& file_name)
{
  FILE *fd = fopen(file_name.c_str(), "w+");
  if (!fd)
  {
    perror("fopen:");
    exit(1);
  }
  for(int i=0; i<x.cols; ++i)
  {
    fprintf(fd,"%f %f %f %f\n", x.at<double>(0,i), x.at<double>(1,i),x.at<double>(2,i),x.at<double>(3,i));
  }
  fclose(fd);
}

void
save2features(const cv::Mat& im1, const cv::Mat& im2, Point2f p1,
              vector<Point2f> neighbors, Point2f best_p2, const string& file_name,
              int lim)
{
  cv::Mat blend, im;
  addWeighted(im1, .6, im2, .4, 0.0, blend);
  cv::cvtColor(blend, im, CV_GRAY2RGB);
  for(int i=0;i<neighbors.size() && i<lim; ++i)
  {
    circle(im, neighbors[i], 1, cv::Scalar(0,255,0), -1);
  }
  circle(im, p1, 2, red);
  circle(im, best_p2, 3, blue);
  cv::imwrite(file_name, im);
}

void
save2blend(const cv::Mat& im1, const cv::Mat& im2, const Mat& x,
           const string& file_name, int lim)
{
  cv::Mat blend, im;
  addWeighted(im1, .5, im2, .5, 0.0, blend);
  cv::cvtColor(blend, im, CV_GRAY2RGB);
  for(int i=0;i<x.cols && i<lim; ++i)
  {
    cv::Point
        p1(x.at<double>(0,i),x.at<double>(1,i)),
        p2(x.at<double>(2,i),x.at<double>(3,i));
    circle(im,p1,1, cv::Scalar(0,255,0),-1);
    circle(im,p2,1, cv::Scalar(0,0,255),-1);
    line(im,p1,p2, cv::Scalar(255,0,0));
  }
  cv::imwrite(file_name, im);
}

void
save2epip(const cv::Mat& im1, const cv::Mat& im2, const Mat& F,
          cv::Point2f pt, const KeyPoints& kp2, const string& file_name)
{
  cv::Mat im_t = vcat<uchar>(im2, im1), im;
  cv::cvtColor(im_t, im, CV_GRAY2RGB);
  circle(im, cv::Point(pt.x, pt.y+im2.rows), 3, cv::Scalar(0,0,255), -1);
  for (int i=0; i<kp2.size(); ++i)
    circle(im, kp2.at(i).pt, 3, cv::Scalar(255,0,0), -1);
  // draw the left points corresponding epipolar lines in right image 
  std::vector<cv::Vec3f> lines;
  vector<cv::Point2f> pts;
  pts.push_back(pt);
  cv::computeCorrespondEpilines(pts, 1, F, lines);
  //for all epipolar lines
  for (vector<cv::Vec3f>::const_iterator it=lines.begin(); it!=lines.end(); ++it)
  {
    // draw the epipolar line between first and last column
    cv::line(im,cv::Point(0,-(*it)[2]/(*it)[1]),
             cv::Point(im2.cols,-((*it)[2]+(*it)[0]*im2.cols)/(*it)[1]),
             cv::Scalar(255,255,255));
  }
  cv::imwrite(file_name, im);
}

void
save4(const Mat& im1, const Mat& im1_prev,
      const Mat& im2, const Mat& im2_prev,
      const KeyPoints& kp1, const KeyPoints& kp1_prev,
      const KeyPoints& kp2, const KeyPoints& kp2_prev,
      const vector<cv::Vec4i>& ind, const string& file_name,
      int lim)
{
  Mat im_t = vcat<uchar>(im1, im1_prev), im_t1 = vcat<uchar>(im2, im2_prev);
  Mat im_t2 = hcat<uchar>(im_t, im_t1);
  cv::Mat im;
  cv::cvtColor(im_t2, im, CV_GRAY2RGB);
  cv::Scalar magenta = cv::Scalar(255, 255,0);
  cv::Scalar yellow = cv::Scalar(0, 255, 255);
  for(int i=0; i<ind.size() && i<lim; ++i)
  {
    cv::Point 
        p1 = kp1[ind.at(i)[0]].pt,
        p2 = kp2[ind.at(i)[1]].pt,
        p3 = kp1_prev[ind.at(i)[2]].pt,
        p4 = kp2_prev[ind.at(i)[3]].pt;
    p2.x += im1.cols;
    p3.y += im1.rows;
    p4.y += im1.rows;
    p4.x += im1.cols;
    line(im, p1, p2, green);
    line(im, p2, p4, green);
    line(im, p4, p3, green);
    line(im, p3, p1, green);
  }
  cv::imwrite(file_name, im);
}

void
saveHarrisCorners(const Mat& harris_response, int thresh, const string& file_name)
{
  Mat dst, dst_norm, dst_norm_scaled;
  cv::Scalar RED = cv::Scalar(0,0,255), color=RED;
  int thickness=5, linetype=1;
  cv::normalize(harris_response, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  Mat im_rgb;
  cv::cvtColor(dst_norm_scaled, im_rgb, CV_GRAY2RGB);
  for( int j = 0; j < dst_norm_scaled.rows ; j++ )
  {
    for( int i = 0; i < dst_norm_scaled.cols; i++ )
    {
      if ((int)dst_norm.at<float>(j,i)>thresh)
      {
        circle(im_rgb, cv::Point(i,j), thickness, color, linetype);
      }
    }
  }
  cv::imwrite(file_name, dst_norm_scaled);
}

void
showHarris(const Mat& harris_response, int thresh)
{
  Mat dst, dst_norm, dst_norm_scaled;
  cv::normalize(harris_response, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, Mat());
  cv::convertScaleAbs(dst_norm, dst_norm_scaled);
  /// Drawing a circle around corners
  for( int j = 0; j < dst_norm_scaled.rows ; j++ )
  {
    for( int i = 0; i < dst_norm_scaled.cols; i++ )
    {
      if ((int)dst_norm.at<float>(j,i)>thresh)
      {
        circle(dst_norm_scaled, cv::Point(i, j), 3,  cv::Scalar(0), 2, 8, 0);
      }
    }
  }
  /// Showing the result
  cv::namedWindow("win", CV_WINDOW_AUTOSIZE );
  cv::imshow("win", dst_norm_scaled);
  cv::waitKey(0);
}
