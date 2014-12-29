#include "myEntropyLib.h"
#include <vector>
using namespace std;
using namespace cv;

const int SIZE_X = 1288;
const int SIZE_Y = 964;
const int ENT_WIN_SIZE = 9;
const int rs = SIZE_X/ENT_WIN_SIZE;
const int cls = SIZE_Y/ENT_WIN_SIZE;
int run = 0;
bool first = true;

vector<Mat> baseHist;
int baseHistSize = 1;
entropyArray arr[rs][cls]={};

void save_hist()
{
  cv::FileStorage fs("my_histogram_file.yml", cv::FileStorage::WRITE);
  if (!fs.isOpened()) {std::cout << "unable to open file storage!" << std::endl; return;}
  fs << "num_of_hist" << baseHistSize;
  for(int i=0;i<baseHistSize;i++)
  {
    Mat my_histogram = baseHist[i];  
    char st[100]={0};
    sprintf(st,"my_histogram_%d",i);
    fs << st << my_histogram;
    
  }
  fs.release();
}

void load_hist()
{
  cv::FileStorage fs("my_histogram_file.yml", cv::FileStorage::READ);
  if (!fs.isOpened()) {std::cout << "unable to open file storage!" << std::endl; return;}
  fs["num_of_hist"] >> baseHistSize;
  cout << baseHistSize << " in file" << endl;
  for (int i=0;i<baseHistSize;i++)
  {
    char st[100]={0};
    sprintf(st,"my_histogram_%d",i);
    Mat hist;
    fs[st] >> hist;
    baseHist.push_back(hist);
  }
  fs.release();
}
void chooseRoad(int event, int x, int y, int flags, void* userdata)
{
  
  if  ( event == EVENT_LBUTTONDOWN )
  {
    Rect rec;
    rec.width = ENT_WIN_SIZE;
    rec.height = ENT_WIN_SIZE;
    rec.x = x - ENT_WIN_SIZE / 2;
    rec.y = y - ENT_WIN_SIZE / 2;
    Mat* ptrImage = (Mat*)userdata;
    Mat sq = (*ptrImage)(rec);
    baseHist.push_back(calculateHistogram(sq));
    baseHistSize++;
    cout << baseHistSize << endl;
  }
  
}
void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
       cout << arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].id << endl;
       printEntro(arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].entropy);
       cout << "idx: " << x/ENT_WIN_SIZE << "  " << y/ENT_WIN_SIZE << endl;
       Rect rec;
       rec.width = ENT_WIN_SIZE;
       rec.height = ENT_WIN_SIZE;
       rec.x = x - ENT_WIN_SIZE / 2;
       rec.y = y - ENT_WIN_SIZE / 2;
       Mat* ptrImage = (Mat*)userdata;
       Mat sq = (*ptrImage)(rec);
       Mat hist = calculateHistogram(sq,"hist of square");
       for(int i=0;i<baseHistSize;i++)
       {
	  double base_test1 = compareHist(baseHist[i],hist,0);
	  cout << base_test1 << endl;
       }
//        imshow("small image",sq);
//        waitKey(0);
      
          //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          run = abs(run - 1);
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
       Rect rec;
       rec.width = ENT_WIN_SIZE;
       rec.height = ENT_WIN_SIZE;
       rec.x = x - ENT_WIN_SIZE / 2;
       rec.y = y - ENT_WIN_SIZE / 2;
       Mat* ptrImage = (Mat*)userdata;
       Mat sq = (*ptrImage)(rec);
       //baseHist = calculateHistogram(sq,"hist of square");
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}


Mat displayMyEntropy(Mat image, int cut_p, int down_p, int toDebug)
{
  Mat sq,empty;
    if(!image.data )
  {    printf( "No image data \n" );     return empty; }
  
  if(!baseHistSize)
  {    
    namedWindow("ent_image", 1);
    
    //set the callback function for any mouse event
    setMouseCallback("ent_image", chooseRoad, (void*)&image);
    imshow("ent_image", image);
    waitKey(0);
    cout << "saving histograms" << endl;
    save_hist();
    return empty;
  }
  else if(first)
  {
   load_hist();
   first = false;
  }
    Rect rec;
    rec.width = ENT_WIN_SIZE;
    rec.height = ENT_WIN_SIZE;
    
    /**BFS the image to find neighbours**/
    
    int id = 1;
    for(int i=0;i<rs;i++)
    {
      for(int j=0;j<cls;j++)
      {
	rec.x = i*ENT_WIN_SIZE;
	rec.y = j*ENT_WIN_SIZE;
	sq = image(rec);
	Mat hist = calculateHistogram(sq);
	int k;
	for(k=0;k<baseHistSize;k++)
	{
	  if(compareHist(baseHist[k],hist,0) > 0.3)
	  {
	    rectangle(image,rec,Scalar(0,0,0),1,8,0);
	    break;
	  }
	}
	if(k == baseHistSize)
	  rectangle(image,rec,Scalar(255,255,255),1,8,0);
      }
    }
    
    /** Paint and show image**/
    if(toDebug)
    {
      
	namedWindow("ent_image", 1);

	//set the callback function for any mouse event
// 	setMouseCallback("ent_image", CallBackFunc, (void*)&image);
	imshow("ent_image", image);
	waitKey(1);
    }    

    return empty;
}