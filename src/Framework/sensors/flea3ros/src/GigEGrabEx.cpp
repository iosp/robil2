// FLEA 3 ROS 		By:Roman Kudinov

//

#include "FlyCapture2.h"
#include <ros/ros.h>
#include <ros/spinner.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>


using namespace FlyCapture2;

image_transport::Publisher publishers[10];

void PrintError( Error error )
{
    error.PrintErrorTrace();
}


void printTriggerModeInfo(TriggerModeInfo info)
{
  printf("present %d\n",info.present);
  printf("read Out Supported: %d\n",info.readOutSupported);
  printf("OnOffSupported: %d\n",info.onOffSupported);
  printf("value readable: %d\n",info.valueReadable);
  printf("mode Mask: %d\n",info.modeMask);
  
}
void printTriggerMode(TriggerMode info)
{
  printf("on off: %d\n", info.onOff);
  printf("polarity: %d\n", info.polarity);
  printf("source: %d\n", info.source);
  printf("mode: %d\n", info.mode);
  printf("parameter: %d\n", info.parameter);

}

void setCamMaster(GigECamera *cam)
{
  StrobeControl s;
  s.source = 2;
  s.onOff = true;
  s.polarity = 0;
  s.delay = 0;
  s.duration = 10;
  Error error = cam->SetStrobe(&s);
  //std::cout << error << std::end;

}
void setCamSlave(GigECamera *cam)
{
  TriggerMode t;
  Error error = cam->GetTriggerMode(&t);
  printTriggerMode(t);
  t.source = 2;
  t.onOff = true;
  t.polarity = 0;
  t.mode = 1;
  t.parameter = 0;
  error = cam->SetTriggerMode(&t);
  error = cam->GetTriggerMode(&t);
  printTriggerMode(t);
}
int RunSingleCamera(PGRGuid guid, int id)
{
    Error error;
    GigECamera cam;
    //GigECamera cam;
    
    

    printf( "Connecting to camera...\n" );
    
    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Get the camera information
    CameraInfo camInfo;
    error = cam.GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    unsigned int numStreamChannels = 0;
    error = cam.GetNumStreamChannels( &numStreamChannels );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    for (unsigned int i=0; i < numStreamChannels; i++)
    {
        GigEStreamChannel streamChannel;
        error = cam.GetGigEStreamChannelInfo( i, &streamChannel );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        printf( "\nPrinting stream channel information for channel %u:\n", i );
        //PrintStreamChannelInfo( &streamChannel );
    }    


    
    // set strobe for id==1
//     if(id == 0)
//     {
//       setCamMaster(&cam);
//     }
//     else
//       setCamSlave(&cam);
    printf( "Querying GigE image setting information...\n" );

    GigEImageSettingsInfo imageSettingsInfo;
    error = cam.GetGigEImageSettingsInfo( &imageSettingsInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    GigEImageSettings imageSettings;
    imageSettings.offsetX = 0;
    imageSettings.offsetY = 0;
    imageSettings.height = imageSettingsInfo.maxHeight;
    imageSettings.width = imageSettingsInfo.maxWidth;
    imageSettings.pixelFormat = PIXEL_FORMAT_RAW8;

    printf( "Setting GigE image settings...\n" );

    error = cam.SetGigEImageSettings( &imageSettings );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
//     if (cameraMasterSlave[0] == guid)
//       setCamMaster(&cam);
//     else if (cameraMasterSlave[1] == guid)
//       setCamSlave(&cam);
    printf( "Starting image capture...\n" );

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    Image rawImage;  
    int seq = 0;
    while(1)
    {              
        // Retrieve an image
        error = cam.RetrieveBuffer( &rawImage );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            continue;
        }
	
	// Copy the data into an OpenCV Mat structure
	cv::Mat bayer8BitMat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC1, rawImage.GetData());
	cv::Mat rgb8BitMat(rawImage.GetRows(), rawImage.GetCols(), CV_8UC3);
	cv::cvtColor(bayer8BitMat, rgb8BitMat, CV_BayerGR2RGB);
// 	cv::imshow("im",rgb8BitMat);
// 	cv::waitKey(1);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rgb8BitMat).toImageMsg();
	msg->header.seq = seq;
	msg->header.frame_id = "1";
	msg->header.stamp = ros::Time::now();
        //msg->height = rgb8BitMat.cols;
	//msg->width = rgb8BitMat.rows;
	//std::cout << rgb8BitMat.cols << rgb8BitMat.rows << std::endl;
	msg->encoding = sensor_msgs::image_encodings::RGB8;
	msg->is_bigendian = 0;
	//msg->step = rawImage.GetStride();

	publishers[id].publish(msg);//CompressMsg(msg));
        seq++;
    }         

    printf( "Stopping capture...\n" );

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }      

    // Disconnect the camera
    error = cam.Disconnect();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    return 0;
}

int main(int argc, char** argv)
{    
    ros::init(argc, argv, "flea3ros");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);
    
    Error error;
    BusManager busMgr;
    CameraInfo camInfo[10];
    unsigned int numCamInfo = 10;
    error = BusManager::DiscoverGigECameras( camInfo, &numCamInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    unsigned int numCameras;
    error = busMgr.GetNumOfCameras(&numCameras);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    for (unsigned int i=0; i < numCameras; i++)
    {
	char topicName[90];
	
	sprintf(topicName, "SENSORS/FLEA3/%d", i);
// 	publishers[i] = n.advertise<sensor_msgs::Image>(topicName, 10);
	publishers[i] = it.advertise(topicName, 10);
        
	PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        InterfaceType interfaceType;
        error = busMgr.GetInterfaceTypeFromGuid( &guid, &interfaceType );
        if ( error != PGRERROR_OK )
        {
            PrintError( error );
            return -1;
        }
//         if (i < 2)
// 	  cameraMasterSlave[i] = guid;
        if ( interfaceType == INTERFACE_GIGE)
	    boost::thread t(RunSingleCamera, guid, i);
	
        
    }

    
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
