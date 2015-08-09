#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
using namespace std;
using namespace cv;

struct Pair
{
   int x;
   int y;
   int fx;
   int fy;
}; 
struct entropyArray
{
  int id;
  vector<double> entropy;
  Mat hist;
};

void printEntro(vector<double> EP)
{
  for (int i=0;i<6;i++)
    cout << EP[i] <<"   ";
  cout << endl;
}

bool compareEntropies(vector<double> EP, vector<double> EP2) //entropyProperties
{
  double AT = 5; //avarageThreshold
  double VT = 5; // varianceThreshold

  if( abs(EP[0]-EP2[0]) > AT || abs(EP[1]-EP2[1]) > AT || abs(EP[2]-EP2[2]) > AT)
    return false;
  else if( abs(EP[3]-EP2[3]) > VT || abs(EP[4]-EP2[4]) > VT || abs(EP[5]-EP2[5]) > VT)
    return false;
  return true;
}
vector<double> calcEntropy(Mat w)
{
  vector<double> res;
  double Ra,Ga,Ba, Rs,Gs,Bs;
  Vec3b intensity;
  for (int i=0; i < w.rows;i++)
  {
    for(int j=0; j < w.cols;j++)
    {
      
      intensity = w.at<Vec3b>(i,j);
      Ra += intensity[0];
      Ga += intensity[1];
      Ba += intensity[2];
    }
  }
  Ra = Ra/(w.rows*w.cols);
  Ga = Ga/(w.rows*w.cols);
  Ba = Ba/(w.rows*w.cols);
  
  for (int i=0; i < w.rows;i++)
  {
    for(int j=0; j < w.cols;j++)
    {
      
      intensity = w.at<Vec3b>(i,j);
      Rs += abs(intensity[0] - Ra);
      Gs += abs(intensity[1] - Ga);
      Bs += abs(intensity[2] - Ba);
    }
  }
  Rs = Rs/(w.rows*w.cols);
  Gs = Gs/(w.rows*w.cols);
  Bs = Bs/(w.rows*w.cols);
  
  res.push_back(Ra);res.push_back(Ga);res.push_back(Ba);
  res.push_back(Rs);res.push_back(Gs);res.push_back(Bs);
  return res;
}

void changeID(entropyArray **arr,int to, int what,int rs,int cls)
{
  for(int i=0;i<rs;i++)
    for(int j=0;j<cls;j++)
      if(arr[i][j].id == what)
	arr[i][j].id = to;
      else if(arr[i][j].id == 0)
	break;
  
}
void printQue(vector<Pair> Q)
{
  for (vector<Pair>::iterator it = Q.begin() ; it != Q.end(); ++it)
    cout << ' ' << (*it).x << ' ' << (*it).y << endl;
  cout << "--------------------" << '\n';
}
bool inQue(vector<Pair> Q,Pair v)
{
  for (vector<Pair>::iterator it = Q.begin() ; it != Q.end(); ++it)
  {
    if((*it).x == v.x && (*it).y == v.y)
      return true;
  }
  return false;
}
int getMin(int v1,int v2)
{
  if(v1 < v2)
    return v1;
  return v2;
}
int getMax(int v1,int v2)
{
  if(v1 > v2)
    return v1;
  return v2;
}

Mat calculateHistogram(cv::Mat const& image,std::string const& name="")
{
    // Set histogram bins count
    //cvtColor(image, image, CV_RGB2GRAY);
    int bins = 256;
    int histSize[] = {bins};
    // Set ranges for histogram bins
    float lranges[] = {0, 256};
    const float* ranges[] = {lranges};
    // create matrix for histogram
    cv::Mat hist;
    int channels[] = {0};

    // create matrix for histogram visualization
    int const hist_height = 256;
    cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);

    cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);
    
    if(!name.empty())
    {
      double max_val=0;
      minMaxLoc(hist, 0, &max_val);

      // visualize each bin
      for(int b = 0; b < bins; b++) {
	  float const binVal = hist.at<float>(b);
	  int   const height = cvRound(binVal*hist_height/max_val);
	  cv::line
	      ( hist_image
	      , cv::Point(b, hist_height-height), cv::Point(b, hist_height)
	      , cv::Scalar::all(255)
	      );
      }
      cv::imshow(name, hist_image);
    }
    normalize( hist, hist, 0, 1, NORM_MINMAX, -1, Mat() );
    return hist;
}

int kMax(int arr[],int size,int k=1)
{
  int I=0,i,max = 9999999;
  for(int j=0;j<k;j++)
  {
    for(I=0,i=1;i<size;i++)  
    {
      if(arr[i] > arr[I] && arr[i] < max)
	I = i;
    }
    max = arr[I];
  }
  return I;
}

Mat createEntropyImage(entropyArray **arr,int rs,int cls,int size,int ID,int sx,int sy)
{
  Mat dest = Mat(sy, sx, CV_8U);
  uchar color;
  
  Rect rec;
  rec.width = size;
  rec.height = size;
  for(int i=0; i< rs; i++)
  {
    for(int j=0; j<cls; j++)
    {
      rec.x = i*size;
      rec.y = j*size;
      if(arr[i][j].id == ID)
	dest(rec).setTo(Scalar(0,0,0));
      else
	dest(rec).setTo(Scalar(255,255,255));
      
    }
  }
  return dest;
}