#include "myEntropyLib.h"
#include <vector>
using namespace std;
using namespace cv;

const int SIZE_X = 1288;//1624;
const int SIZE_Y = 964;//1224;
const int ENT_WIN_SIZE = 35;
const int rs = SIZE_X/ENT_WIN_SIZE;
const int cls = SIZE_Y/ENT_WIN_SIZE;
int run = 1;
Mat baseHist;
struct MouseParams
{
    Mat *img;
    entropyArray **arr;
};
void CallBackFunc(int event, int x, int y, int flags, void* param)
{
     MouseParams* mp = (MouseParams*)param;
     Mat* img = mp->img;
     if  ( event == EVENT_LBUTTONDOWN )
     {
       
       entropyArray** arr = mp->arr;
       cout << "id of cell: " << arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].id << endl;
       printEntro(arr[x/ENT_WIN_SIZE][y/ENT_WIN_SIZE].entropy);
       cout << "idx: " << x/ENT_WIN_SIZE << "  " << y/ENT_WIN_SIZE << endl;
       Rect rec;
       rec.width = ENT_WIN_SIZE;
       rec.height = ENT_WIN_SIZE;
       rec.x = x - ENT_WIN_SIZE / 2;
       rec.y = y - ENT_WIN_SIZE / 2;
       Mat sq = (*img)(rec);
       Mat hist = calculateHistogram(sq,"hist of square");
       if (!baseHist.data)
	 baseHist = calculateHistogram(sq);
       double base_test1 = compareHist(baseHist,hist,0);
       cout << "compared to base: " << base_test1 << endl;
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
       Mat sq = (*img)(rec);
       baseHist = calculateHistogram(sq,"hist of square");
       cout << "base hist chosen" << endl;
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
    vector<double> tempEnt = calcEntropy(image);
    Rect rec;
    rec.width = ENT_WIN_SIZE;
    rec.height = ENT_WIN_SIZE;
    entropyArray **arr;
//     int rs = image.cols/ENT_WIN_SIZE;
//     int cls = image.rows/ENT_WIN_SIZE;
    arr = new entropyArray*[rs];
    for(int i = 0; i < rs; ++i)
      arr[i] = new entropyArray[cls];
    
    
//     cout << "rows : " << rs << " cols: " << cls << "  " << image.rows <<"  " << image.cols << endl;
    
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
	arr[i][j].hist = calculateHistogram(sq);
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
    /**Perform Entropy calculations on image**/
    while(!Q.empty())
    {
      /**Choose first in Q**/
      t = Q.back();
      Q.pop_back();
      rec.x = t.x*ENT_WIN_SIZE;
      rec.y = t.y*ENT_WIN_SIZE;
      
      /** Add Id to cell **/
      if((compareEntropies(arr[t.fx][t.fy].entropy,arr[t.x][t.y].entropy)) || 
				  compareHist(arr[t.fx][t.fy].hist,arr[t.x][t.y].hist,0) > 0.75 )
	arr[t.x][t.y].id = arr[t.fx][t.fy].id;
      else
      {
	arr[t.x][t.y].id = id;
	id++;
      }
	
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
	  changeID(arr,getMin(arr[t.x][t.y].id,arr[n.x][n.y].id),getMax(arr[t.x][t.y].id,arr[n.x][n.y].id),rs,cls);	  
      
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
    /** count The IDs and find the best ID **/
    int idCtr[rs*cls]={0};
    int I_max = 0,I_max2 = 1;
    for(int i=0;i<rs;i++)
	for(int j=0;j<cls;j++)
	  idCtr[arr[i][j].id]++;
    int ID = kMax(idCtr,rs*cls,2);
    //empty = createEntropyImage(arr,rs,cls,ENT_WIN_SIZE,ID,SIZE_X,SIZE_Y);
    /** Paint and show image**/
    if(toDebug)
    {
//       for(int i=0;i<rs*cls;i++)
// 	if (idCtr[i] && idCtr[i]>10)
// 	  cout << "id " << i << ": "<< idCtr[i] <<endl;
//       cout << "2nd is: " << ID << endl;
      int max_clr = 255 / id; 
      int clr = 0;
      for(int i=0;i<rs;i++)
	for(int j=0;j<cls;j++)
	{
	  rec.x = i*ENT_WIN_SIZE;
	  rec.y = j*ENT_WIN_SIZE;
	  if (arr[i][j].id == ID)
	    clr = 0;
	  else
	    clr = 18;
	  clr = (arr[i][j].id) % 255;
	  //cout << arr[i][j].id << "   " << endl;printEntro(arr[i][j].entropy);
	  rectangle(image,rec,Scalar(((clr*5)%255),((clr*9)%255),((clr*12)%255)),1,8,0);
	  
	}
	
	namedWindow("ent_image", 1);

	//set the callback function for any mouse event
        MouseParams param;
	param.img = &image;
	param.arr = arr;
	setMouseCallback("ent_image", CallBackFunc, (void*)&param);
	imshow("ent_image", image);
	//imshow("entropy_mask", empty);
	waitKey(run);
    }    
    
    for(int i = 0; i < rs; ++i)
      delete [] arr[i];
    delete [] arr;
    
    return empty;
}