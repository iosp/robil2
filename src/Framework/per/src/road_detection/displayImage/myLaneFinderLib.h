int f_id = 0;
typedef struct polydat{
  vector<double> ps;
  Point p1;
  Point p2;
}polydat;

Mat DilationErosion(Mat src, double factor,char chr)
{
  /*
   * This function performs dilation or erosion.
   * If chr input is 'e' or 'E' erosion
   * else dilation
   */
  Mat dst;
  
  int erosion_size=2, erosion_type = MORPH_RECT;
  Mat element = getStructuringElement( erosion_type,
				      Size( factor*erosion_size + 1, factor*erosion_size+1 ),
				      Point( erosion_size, erosion_size ) );


  /// Apply the Dilation operation
  if (chr == 'e' || chr == 'E')
    erode( src, dst, element );
  else
    dilate( src, dst, element );
  return dst;
}

vector<Point> get_coordinates(Mat *image)
{
  /*
   * This function gets the none zero coordinates from a binary image.
   * This function replaces the 'find' function performed in MATLAB
   */
  vector<Point> myvector;
  Point p;
  for(int i=0; i< image->cols; i++)
    for(int j=0; j < image->rows; j++)
    {
      if (image->at<uchar>(j,i) == 255)
      {
	
	p.x = i;
	p.y = j;
	myvector.push_back(p);
	//image->at<uchar>(j,i) = 255;
      }
      else
	//image->at<uchar>(j,i) = 0
	;
    }
  return myvector;
}

template<typename T> vector<T> subvector( const vector<T>& vec, int i, int j)
{
  if( j > vec.size())
    j = vec.size();
  vector<T> subvec;
  for(;i<j;i++)
    subvec.push_back(vec[i]);
  return subvec;
}

vector<polydat> runPolyfit(vector<Point> points)
{
  /*
   * This function runs the polyfit function and returns the 2nd order polynom that runs through points.
   */
  int POLYRANK = 2;
  vector<polydat> pdat;
  vector<double> xs, ys;
  double alpha;
  if(points.size() < 10)
      return pdat;
  /*
   * clean useless points
   */
  for (vector<Point>::iterator it = (points.begin()+1) ; it != points.end(); ++it)
  {
    if( (*it).x == (*(it-1)).x )
      continue;
    alpha = ( (*it).y - (*(it-1)).y ) / ( (*it).x - (*(it-1)).x );
    if( abs(alpha) > 10 || abs(alpha) < 0.5 )
      continue;
    xs.push_back((*it).x);
    ys.push_back((*it).y);
  }
  int N = xs.size();
  if(N < 40)
  {
    return pdat;
  }
  int I = 0;
  int ma_size = 5;
  for(int i = ma_size; i < (xs.size()-1); i++)
  {
    vector<double> x_temp = subvector(xs,i-ma_size,i); vector<double> y_temp = subvector(ys,i-ma_size,i);
    if (x_temp.size() < 3) continue;
    vector<double> p = polyfit(x_temp,y_temp,POLYRANK);
    vector<double> y_est = polyval(p, subvector(xs,i,i+1));
    if (abs(y_est[0] - ys[i]) > 35)
    {
      polydat dat;
      vector<double> x_temp = subvector(xs,I,i-1); vector<double> y_temp = subvector(ys,I,i-1);
      if(x_temp.size() < 3) continue;
      dat.ps = polyfit(x_temp,y_temp,POLYRANK);
      dat.p1.x = subvector(xs,I,I+1)[0];dat.p1.y = subvector(ys,I,I+1)[0];
      dat.p2.x = subvector(xs,i-1,i)[0];dat.p2.y = subvector(ys,i-1,i)[0];
      pdat.push_back(dat);
      I = i;
    }
  }
  return pdat;    
}
