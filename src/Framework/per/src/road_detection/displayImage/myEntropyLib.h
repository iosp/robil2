#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>

using namespace std;
using namespace cv;

#define ENT_WIN_SIZE 9
double entropyPrecalc[ENT_WIN_SIZE * ENT_WIN_SIZE + 1];
void calculate_entropy_values()
{
  double sum = ENT_WIN_SIZE*ENT_WIN_SIZE+1;
  for (int i=0; i<sum;i++)
    if (i == 0) 
      entropyPrecalc[i] = 0;
    else 
      entropyPrecalc[i] = -(i / sum) * log(i / sum);
}
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
  double entropy;
  //int* hist;
};

void printEntro(vector<double> EP)
{
  for (int i=0;i<6;i++)
    cout << EP[i] <<"   ";
  cout << endl;
}
//ask oded how do i compare enthrophies and hist

bool compareEntropies(double EP, double EP2, double AT) //entropyProperties
{
  return (abs(EP-EP2)<AT);
}

bool compareHist;

void calculateHistogram(Mat* image, int *values){
  int  i, j;

  
 
  for(i=0; i<image->rows; i++)
    for(j=0; j<image->cols; j++) {
      Scalar colour = image->at<uchar>(Point(j, i));
      values[(int)colour.val[0]]+=1; //increments the coressponding value in the array **CHECKED**
    }
}

double calcEntropy(Mat image)
{
  int  i, values[256];
  double res=0;
  for(i=0; i < 256 ; i++)
    values[i]=0;
  calculateHistogram(&image, values);
  for(int i = 0; i<256; i++)
  {
    int N = values[i];
    if(N > 35*35) {
      res = 0;
      break;
    }
    res += entropyPrecalc[N];
  }

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



int kMax(int arr[],int size,int k=1)
{
  /*
   * Finds the index with the largest components
   */
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
  /*
   * Create an image with the given index
   */
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
      if(arr[i][j].id != ID)
	dest(rec).setTo(Scalar(0,0,0));
      else
	dest(rec).setTo(Scalar(255,255,255));
      
    }
  }
  return dest;
}
