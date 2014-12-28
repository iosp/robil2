#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
using namespace std;
using namespace cv;
int run = 1;

Mat calculateHistogram(cv::Mat const& image,std::string const& name="")
{
    // Set histogram bins count
    int bins = 10;
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
    return hist;
}


double calculateEntropy()
{
  
}


Mat displayMyEntropy(Mat image, int cut_p, int down_p, int toDebug)
{
    Mat sq,empty,gr;
    if(!image.data )
    {    printf( "No image data \n" );     return empty; }
    cvtColor(image, image, CV_BGR2GRAY);
    calculateHistogram(image,"hist");
    
    imshow("ent_image", image);
    waitKey(run);

    return empty;
}