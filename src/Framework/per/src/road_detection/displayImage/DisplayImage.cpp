#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <queue>
#include "polyfit.h"

//#include <unistd.h>
//unsigned int microseconds = 5000000;

using namespace std;
using namespace cv;

vector<double> entropyArray;
vector<double>  roadMatrix;

int im_rows,im_cols;
float win_height, win_width;

const float WIN_SIZE = 85.0;

int cut_percents;
int down_percents;

Rect getROI(int l, int k);

void printEntropy()
{ 
  for(int i = 0; i < WIN_SIZE*WIN_SIZE; i++) if(entropyArray.at(i)) printf("i: %d - %f\n", i+1, entropyArray.at(i));
}

vector<double> getEntropy(Mat w, int l, int k, int cut_percents, int down_percents)
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

/**
 * Building the image block by block according to entropyArray 
 */

Mat createEntropyImage(Mat m)
{
  Mat dest = Mat(im_rows, im_cols, CV_8U);
  uchar color;
  
  Rect roi;
  int l,k;
  for(l=0; l< WIN_SIZE; l++)
  {
    for(k=0; k<WIN_SIZE; k++)
    {	
      roi = getROI(l,k);
      color = entropyArray[l*WIN_SIZE + k];
      dest(roi).setTo(Scalar(color));
      
    }
  }
  return dest;
}



/** *********************
	Erode
 ** *********************/

Mat Erosion(Mat src, double factor)
{
  Mat erosion_dst;
//   int erosion_type = MORPH_ELLIPSE;
  int erosion_type = MORPH_RECT;
  Mat element = getStructuringElement( erosion_type,
                                       Size( factor*win_width+1, factor*win_height +1),
                                       Point( win_width, win_height ) );

  /// Apply the erosion operation
  erode( src, erosion_dst, element );
  return erosion_dst;
}


/** *********************
	Dilate
 ** *********************/

Mat Dilation(Mat src, double factor)
{
  Mat Dilation_dst;
//   int dilation_type = MORPH_ELLIPSE;
  int dilation_type = MORPH_RECT;
  
  Mat element = getStructuringElement( dilation_type,
                                       Size( factor*win_width+1, factor*win_height +1),
                                       Point( win_width, win_height ) );

  /// Apply the Dilation operation
  dilate( src, Dilation_dst, element );
  return Dilation_dst;
}



/** *******************************
 *  findLimits :
 * finds the leftmost and rightmost points of the comtour describing the road
 * and put them inside LL and LR respectively
 ** *******************************/

void findLimits(vector<Point> *LR, vector<Point> *LL, vector<vector<Point> > *lanes, vector<Point> cont)
{
  int cont_size = cont.size();
  
  int selectedIdxL = 0;
  int direction = 0,counts = 1;
  for(; direction == 0 ; counts++)
  {
    if(cont[(selectedIdxL+counts)%cont_size].y > cont[selectedIdxL].y)
    {
      direction = 1;
    }
    else if(cont[(selectedIdxL+counts)%cont_size].x > cont[selectedIdxL].y)
    {
      direction = -1;
    }
  }
  
  bool goes_down = false; 
  int PointsToCount = 30;
  int counter = 0;
  
  for(int i= 0+direction; !goes_down; i+=direction)
  {  
    if(counter == PointsToCount || i%cont_size==0)
    {
      goes_down = true;
    }
    else 
    {
      counter++;
      if(cont[(i%cont_size)].x < im_cols/2)
	LL->push_back(cont[(i%cont_size)]);
    }
  }
  
  goes_down = false;
  for(int i= counter; !goes_down; i+=direction)
  {  
    if(i%cont_size==0)
    {
      goes_down = true;
    }
    else 
    {
      counter++;
       if(cont[(i%cont_size)].x >im_cols/2)
	LR->push_back(cont[(i%cont_size)]);
    }
  }
}

/**
 * returns whether two vectors describing the properties of two blocks are 
 * similar enaough according to predecided thresholds
 * AT - represents the maximal difference between the avarage  of RGB values of the vectors
 * VT - represents the maximal difference between the variance of RGB values of the vectors
 */
bool compareEntropies(vector<double> EP, vector<double> EP2) //entropyProperties
{
  double AT = 15; //avarageThreshold
  double VT = 15; // varianceThreshold
  
  if( abs(EP[0]-EP2[0]) > AT || abs(EP[1]-EP2[1]) > AT || abs(EP[2]-EP2[2]) > AT)
    return false;
  else if( abs(EP[3]-EP2[3]) > VT || abs(EP[4]-EP2[4]) > VT || abs(EP[5]-EP2[5]) > VT)
    return false;
  return true;
}

/**
 * returns a Rectangle  describing the (l,k) block's limits
 */
Rect getROI(int l, int k)
{
  int j, i, jPlus, iPlus;
  i = l*win_height;
  j = k*win_width;
    
  if ((i+win_height >= im_rows) && (j+win_width >= im_cols))
  {
    iPlus = im_rows-i-1;
    jPlus = win_width;
  }
  else if(j+win_width >= im_cols)
  {
    jPlus = im_cols-j-1;
    iPlus = win_height;
  }
  else if(i+win_height >= im_rows)
  {
    iPlus = im_rows-i-1;
    jPlus = im_cols-j-1;
  }
  else
  {
    iPlus = win_height;
    jPlus = win_width;
  }
  Rect roi = Rect(j,i,jPlus,iPlus);
  return roi;
}

/**
 * returns the sub_image inside the block (l,k)
 */
Mat getROIByLandK(Mat* image, int l, int k)
{
  Rect roi = getROI(l,k);  
  return Mat(*image, roi);
}

/**
 * Pair structure :
 * holds pair of blocks 
 * (l,k) and (Fl,Fk) which is (l,k)'s father block in the BFS
 */
struct Pair
{
   int l;
   int k;
   int Fl;
   int Fk;
}; 

/**
 * calculate the similarity of a block (l,k) to the father block (Fl,Fk)
 */
bool calcEntropyFor_L_and_k(Mat* m, int l, int k, int fl, int fk)
{
  /// check entropy for this slot: 
  Mat w = getROIByLandK(m, l, k);
  vector<double> ep = getEntropy(w, l, k, cut_percents, down_percents);
  
  Mat fw = getROIByLandK(m, fl, fk);
  vector<double> fep = getEntropy(fw, fl, fk, cut_percents, down_percents);
  
  if(compareEntropies(ep, fep))
  {
   return (entropyArray[l*WIN_SIZE +k] = 255);
  }
  else
    return 0;//(entropyArray[l*WIN_SIZE +k] = 0);
}

bool comparePairs(Pair p, Pair q)
{
  if(p.l == q.l && p.k == q.k)
    return true;
  return false;
}

bool pairVectorContains(vector<Pair> *pairs, Pair p)
{
  for(vector<Pair>::iterator it = pairs->begin(); it != pairs->end(); ++it) 
  {
    if(comparePairs(*it,p))
      return true;
  }
  return false;
}

/**
 * findRoadStartFromLK :
 * BFS the image starting from block (l,k) 
 * in order to find the road from this block and so on
 **/ 

void findRoadStartFromLK(Mat *image, int l, int k)
{
  vector<Pair> Q;
  
  ///check limits:
  if(l<0 || l == WIN_SIZE)
    return;
  /// mark the vertex as visited
  roadMatrix[l*WIN_SIZE +k] = 1;
  
  Pair p = {l,k,l,k};
  vector<Pair>::iterator it;
  it = Q.begin();
  it = Q.insert(it, p);
  
  Pair t;
  while(!Q.empty())
  {
    t = Q.back();
    Q.pop_back();
    
    if(calcEntropyFor_L_and_k(image, t.l, t.k, t.Fl, t.Fk) )
    {
      if( t.l>0 && roadMatrix[(t.l-1)*WIN_SIZE +t.k] != 1) // upper neighbor
      {
	roadMatrix[(t.l-1)*WIN_SIZE +t.k] = 1;
	Pair PP = {t.l-1, t.k, t.l, t.k};
	it = Q.begin();
	it = Q.insert(it, PP);
      }
      if( t.l<WIN_SIZE-1 && roadMatrix[(t.l+1)*WIN_SIZE +t.k] != 1) // lower neighbor
      {
	roadMatrix[(t.l+1)*WIN_SIZE +t.k] = 1;
	Pair PP = {t.l+1, t.k, t.l, t.k};
	it = Q.begin();
	it = Q.insert(it, PP);
      }
      if(t.k != WIN_SIZE-1 && roadMatrix[t.l*WIN_SIZE +(t.k+1)] != 1) // right neighbor
      {
	roadMatrix[t.l*WIN_SIZE +(t.k+1)] = 1;
	Pair PP = {t.l, t.k+1, t.l, t.k};
	it = Q.begin();
	it = Q.insert(it, PP);
      }
      if(t.k != 0 && roadMatrix[(t.l)*WIN_SIZE + (t.k-1)] != 1) // left neighbor
      {
	roadMatrix[(t.l)*WIN_SIZE + (t.k-1)] = 1;
	Pair PP = {t.l, t.k-1, t.l, t.k};
	it = Q.begin();
	it = Q.insert(it, PP);
      }
    }
  }

}


/** *******************************
 *  detectRoad:
 ** *******************************/
vector<double> detectRoad(Mat image, int cut_p, int down_p, int toDebug) 
{  
  vector<double> result;
  if(!image.data )
  {    printf( "No image data \n" );     return result; }
  
  im_rows = image.rows; 
  im_cols = image.cols;
  
  win_height = im_rows/WIN_SIZE;
  win_width  = im_cols/WIN_SIZE;
  
  entropyArray.clear();
  roadMatrix  .clear();
  entropyArray.resize(WIN_SIZE*WIN_SIZE, 0);
  roadMatrix  .resize(WIN_SIZE*WIN_SIZE, 0);
  
  Mat w;
  Rect roi;
  cut_percents = cut_p; 
  down_percents = down_p;
  
  
  findRoadStartFromLK(&image, WIN_SIZE-10, WIN_SIZE*3/4);
  findRoadStartFromLK(&image, WIN_SIZE-10, WIN_SIZE*1/4);
  findRoadStartFromLK(&image, WIN_SIZE-20, WIN_SIZE*2/3);
  findRoadStartFromLK(&image, WIN_SIZE-20, WIN_SIZE*1/3);
  
  if(toDebug)
  {
    Rect ROI = getROI(WIN_SIZE-10, WIN_SIZE*3/4);
    rectangle(image, Point(ROI.x,ROI.y), Point( ROI.x-ROI.width,ROI.y-ROI.height), Scalar(0,0,255),2,8,0);
    
    ROI = getROI(WIN_SIZE-10, WIN_SIZE*1/4);
    rectangle(image, Point(ROI.x,ROI.y), Point( ROI.x-ROI.width,ROI.y-ROI.height), Scalar(0,0,255),2,8,0);
    
    ROI = getROI(WIN_SIZE-20, WIN_SIZE*2/3);
    rectangle(image, Point(ROI.x,ROI.y), Point( ROI.x-ROI.width,ROI.y-ROI.height), Scalar(0,0,255),2,8,0);
    
    ROI = getROI(WIN_SIZE-20, WIN_SIZE*1/3);
    rectangle(image, Point(ROI.x,ROI.y), Point( ROI.x-ROI.width,ROI.y-ROI.height), Scalar(0,0,255),2,8,0);
  } 
  
///create entropy image  
   Mat entropyImage = createEntropyImage(image);
   
/** *****************************************
 * finding the actual road : 
 * *******************************************/
  
/// eliminate noise on the road: 

  int factor = 2;
  entropyImage = Dilation(entropyImage, factor);
  factor = 2;
  entropyImage = Erosion(entropyImage, factor);

  imshow("debug", entropyImage);
  
/// finding clusters:
  vector<vector<Point> > contours;
  vector<vector<Point> > selected_contours;
  vector<Vec4i> hierarchy;  
  
  findContours( entropyImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
  
  int largest=0, second;
  double max_area=0, second_area=0, curr_area=0;
  int selected_contour;
  
  for( int k = 0; k < contours.size(); k++ )
  {
    curr_area = contourArea(contours[k]);
    
    if(curr_area > max_area)
    {
      second = largest;
      second_area = max_area;
      
      max_area = curr_area;      
      largest = k;
    }
    else if( curr_area > second_area)
    {
      second_area = curr_area;
      second = k;
    }
    /// take the big ones:
    if( (contourArea(contours[k]) > 10000 && hierarchy[k][3] == -1) )
    {
      selected_contours.push_back(contours[k]);
      selected_contour = k;
      
      if(toDebug)
	drawContours(image, contours, k, Scalar(255,255,255), 2, 8);
    }
    
  }
  
  int s = selected_contours.size();
  
  for(int i=0; i<s; i++)
  {    
    vector<Point> limitsR;
    vector<Point> limitsL;
    vector<vector<Point> > lanes;
    findLimits(&limitsR, &limitsL, &lanes, selected_contours[i]);
    
    double a[3];
    polyfit(a, limitsL, limitsL.size()-1);
    
    double b[3];
    polyfit(b, limitsR, limitsR.size()-1);
    
    vector< double*> Cs;
    for(int j=0; j<lanes.size(); j++)
    {
      double c[3];
      polyfit(c, lanes[j], lanes[j].size());
      Cs.push_back(c);
    }
    
    if(a[0] == a[0] && a[1] == a[1] && a[2] == a[2])
    {
      result.push_back(limitsL[0].y);
      result.push_back(limitsL[limitsL.size()-1].y);
      result.push_back(a[2]);
      result.push_back(a[1]);
      result.push_back(a[0]);
    }
    else
    {
      result.push_back(0);
      result.push_back(0);      
      result.push_back(0);
      result.push_back(0);
      result.push_back(0);
    }
    
    for(int j=0; j<Cs.size();j++)
    {
      if(Cs[j][0] == Cs[j][0] && Cs[j][1] == Cs[j][1] && Cs[j][2] == Cs[j][2])
      {
	result.push_back(lanes[j][0].y);
	result.push_back(lanes[j][lanes[j].size()-1].y);
	result.push_back(Cs[j][2]);
	result.push_back(Cs[j][1]);
	result.push_back(Cs[j][0]);
      }
      else
      {
	result.push_back(0);
	result.push_back(0);
	result.push_back(0);
	result.push_back(0);
	result.push_back(0);
      } 
    }
    
    if(b[0] == b[0] && b[1] == b[1] && b[2] == b[2])
    {
      result.push_back(limitsR[0].y);
      result.push_back(limitsR[limitsR.size()-1].y); 
      result.push_back(b[2]);
      result.push_back(b[1]);
      result.push_back(b[0]);
    }
    else
    {
      result.push_back(0);
      result.push_back(0);      
      result.push_back(0);
      result.push_back(0);
      result.push_back(0);
    }
    
    /// debug purposes: 
    if(toDebug)
    {
      printf("\nSTART DEBUG\n");
      
      int limLs = limitsL.size() > 0 ? limitsL.size()-1 : 0; 
      int limRs = limitsR.size() > 0 ? limitsR.size()-1 : 0; 
      
      for(int j=0; j<limLs; j++)
      {
	int x = limitsL[j].y;
	double y = a[0] + a[1]*x*1.0 + a[2]*x*x*1.0;
	
	circle(image, Point(y,x), 3, Scalar(0,0,255), -1, 8, 0);	
      }
      
      for(int j=0; j<limRs ; j++)
      {
	int x = limitsR[j].y;
	double y = b[0] + b[1]*x*1.0 + b[2]*x*x*1.0;
	
	circle(image, Point(y,x),3, Scalar(255,0,0), -1, 8, 0);
	
// 	circle(image, limitsR[j],3, Scalar(255,0,255), -1, 8, 0);    
      }

      
      for(int j=0; j<lanes.size(); j++)
      {
	for(int k=0; k<lanes[j].size(); k++)
	{
	  if( &lanes[j][k] != 0)
	  {
	    int x = lanes[j][k].y;
	    double y = Cs[j][0] + Cs[j][1]*x*1.0 + Cs[j][2]*x*x*1.0;
	    
	    circle(image, Point(y,x), 3, Scalar(120*(j+1),120*(j+1),0), -1, 8, 0);
	  }
	}
      }
      printf("END DEBUG\n");
    }
  }
  
  if(toDebug)
  {
    imshow("walrus", image);
    waitKey(1);
  }
  
    
  return result;
}


