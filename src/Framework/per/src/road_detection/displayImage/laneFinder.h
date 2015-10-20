#ifndef LANEFINDER_H
#define LANEFINDER_H

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "polyfit.h"
#include "myLaneFinderLib.h"



vector<polydat> find_lanes(Mat ent_image, int toDebug)
{
  Rect rec;
  int pix_to_remove = 20;
  rec.width = ent_image.cols - 2 * pix_to_remove;
  rec.height = ent_image.rows - 2 * pix_to_remove;
  rec.x = pix_to_remove;
  rec.y = pix_to_remove;
  ent_image = ent_image(rec);
  int factor = 5;
  ent_image = DilationErosion(ent_image, factor,'d');
  ent_image = DilationErosion(ent_image, factor,'e');
  Mat detected_edges;
  int lowThreshold = 20;
  Canny( ent_image, detected_edges, lowThreshold, lowThreshold*3, 3 );
  vector<Point> myvector = get_coordinates(& detected_edges);
  vector<polydat> ps = runPolyfit(myvector);
  if(toDebug)
  {
    for (vector<polydat>::iterator it=ps.begin(); it != ps.end(); it++)
      line( detected_edges, (*it).p1, (*it).p2, Scalar(255,0,0), 2, 8, 0);
    imshow("contours", detected_edges);    
    waitKey(1);
  }
  return ps;
}


//  Mat detected_edges;
//     int lowThreshold = 20, erosion_size=2, erosion_type = MORPH_RECT;
//     Mat element = getStructuringElement( erosion_type,
//                                        Size( 2*erosion_size + 1, 2*erosion_size+1 ),
//                                        Point( erosion_size, erosion_size ) );
//     
//     
//     dilate( empty, empty, element );dilate( empty, empty, element );
//     dilate( empty, empty, element );dilate( empty, empty, element );
// //     Canny( empty, detected_edges, lowThreshold, lowThreshold*3, 3 );
// 
//     vector<Point> myvector = get_coordinates(&detected_edges);
//     FILE *f = fopen("pointsTest.txt","w");
//     for (vector<Point>::iterator it = myvector.begin() ; it != myvector.end(); ++it)
//        fprintf(f,"%d,%d\n",(*it).x,(*it).y);
//     fclose(f);

//      imshow( "canny", detected_edges );
//       vector<int> compression_params;
//       compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
//       compression_params.push_back(9);
//       imwrite("entropy_mask.png",empty, compression_params);
//       waitKey(0);
      
#endif // LANEFINDER_H
