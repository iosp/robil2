#include "DisplayImage.h"
#include <vector>
using namespace std;
using namespace cv;

const int SIZE_X = 1288;
const int SIZE_Y = 964;
const int ENT_WIN_SIZE = 9;
const int rs = SIZE_X/ENT_WIN_SIZE;
const int cls = SIZE_Y/ENT_WIN_SIZE;
int run = 1;

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
};
entropyArray arr[rs][cls]={};

void printEntro(vector<double> EP)
{
  for (int i=0;i<6;i++)
    cout << EP[i] <<"   ";
  cout << endl;
}

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  ( event == EVENT_LBUTTONDOWN )
     {
       cout << arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].id << endl;
       printEntro(arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].entropy);
       cout << "idx: " << x/ENT_WIN_SIZE << "  " << y/ENT_WIN_SIZE << endl;
          //cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if  ( event == EVENT_RBUTTONDOWN )
     {
          //cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          run = abs(run - 1);
     }
     else if  ( event == EVENT_MBUTTONDOWN )
     {
          //cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
     }
     else if ( event == EVENT_MOUSEMOVE )
     {
          //cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;

     }
}

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

void changeID(int to, int what)
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
Mat displayMyEntropy(Mat image, int cut_p, int down_p, int toDebug)
{
    Mat sq,empty;
    if(!image.data )
  {    printf( "No image data \n" );     return empty; }
    vector<double> tempEnt = calcEntropy(image);
    Rect rec;
    rec.width = ENT_WIN_SIZE;
    rec.height = ENT_WIN_SIZE;
    
    //cout << "rows : " << rs << " cols: " << cls << "  " << image.rows <<"  " << image.cols << endl;
    
    /**BFS the image to find neighbours**/
    
    int id = 1;
    
    for(int i=0;i<rs;i++)
    {
      for(int j=0;j<cls;j++)
      {
	rec.x = i*ENT_WIN_SIZE;
	rec.y = j*ENT_WIN_SIZE;
	sq = image(rec);
	arr[i][j].entropy = calcEntropy(sq);
	arr[i][j].id = 0;
      }
    }
    arr[0][0].id = id;
    vector<Pair> Q;
    Pair p1 = {0,1,0,0};
    Pair p2 = {1,0,0,0};
    vector<Pair>::iterator it;
    it = Q.begin();
    it = Q.insert(it,p1);
    it = Q.begin();
    it = Q.insert(it,p2);
    
    Pair t;
    while(!Q.empty())
    {
      /**Choose first in Q**/
      t = Q.back();
      Q.pop_back();
      rec.x = t.x*ENT_WIN_SIZE;
      rec.y = t.y*ENT_WIN_SIZE;
      
      /** Add Id to cell **/
      if(!(compareEntropies(arr[t.fx][t.fy].entropy,arr[t.x][t.y].entropy)))
      {
	arr[t.x][t.y].id = id;
	id++;
      }
      else
	arr[t.x][t.y].id = arr[t.fx][t.fy].id;
      Pair n;
      if(t.fx == t.x)
      {
	n.x = t.x-1;
	n.y = t.y;
      }
      else
      {
	n.x = t.x;
	n.y = t.y-1;
      }
      if(n.x >= 0 && n.y >= 0 && arr[t.x][t.y].id != arr[n.x][n.y].id)
	if((compareEntropies(arr[t.x][t.y].entropy,arr[n.x][n.y].entropy)))
	  changeID(getMin(arr[t.x][t.y].id,arr[n.x][n.y].id),getMax(arr[t.x][t.y].id,arr[n.x][n.y].id));	  
      
      /**Add sons to Que**/
      Pair P1 = {t.x+1,t.y,t.x,t.y};
      if(!(t.x+1 >= rs) && arr[t.x+1][t.y].id == 0 && !inQue(Q,P1))
      {
	it = Q.begin();
	it = Q.insert(it,P1);
      }
      Pair P2 = {t.x,t.y+1,t.x,t.y};
      if(!(t.y+1 >= cls) && !arr[t.x][t.y+1].id && !inQue(Q,P2))
      {
	it = Q.begin();
	it = Q.insert(it,P2);
      }
    }
    /** Paint and show image**/
    int max_clr = 255 / id; 
    int clr = 0;
    for(int i=0;i<rs;i++)
      for(int j=0;j<cls;j++)
      {
	rec.x = i*ENT_WIN_SIZE;
	rec.y = j*ENT_WIN_SIZE;
	clr = max_clr*arr[i][j].id;
	//cout << arr[i][j].id << "   " << endl;printEntro(arr[i][j].entropy);
	rectangle(image,rec,Scalar(((arr[i][j].id*5)%255),((arr[i][j].id*9)%255),((arr[i][j].id*12)%255)),1,8,0);
	
      }
      namedWindow("ent_image", 1);

      //set the callback function for any mouse event
     setMouseCallback("ent_image", CallBackFunc, arr);
    imshow("ent_image", image);
    waitKey(run);
//     for(int i=0;i<rs;i++)
//     {
//       for(int j=0;j<cls;j++)
//       {
// 	rec.x = i*ENT_WIN_SIZE;
// 	rec.y = j*ENT_WIN_SIZE;
// 	sq = image(rec);
// 	vector<double> ent = calcEntropy(sq);
// 	if(!compareEntropies(ent,tempEnt))
// 	  clr = (clr+30)%255;
// 	rectangle(image,rec,Scalar(100,100,clr),1,8,0);
// 	imshow("ent_image", image);
// 	tempEnt = ent;
// 	//imshow("cropped",sq);
// 	waitKey(0);
//       }
//     }
    
//     if(toDebug)
//     {
//       imshow("reg_image", image);
//       waitKey(0);
//     }
    

    return empty;
}