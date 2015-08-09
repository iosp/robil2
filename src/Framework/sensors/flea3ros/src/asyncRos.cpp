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

#define SOFTWARE_TRIGGER_CAMERA

const unsigned int  SLEEP_MICRO_SEC = 5;
const unsigned int  CAP_RATE = 35;
bool firecam[2] = {false,false};

using namespace FlyCapture2;

image_transport::Publisher publishers[10];

void fireThread(unsigned int numCameras)
{
  while(1)
  {
    while(firecam[0] || firecam[1])
      usleep(SLEEP_MICRO_SEC * 1000);
    for (int i=0;i<2;i++)
      firecam[i] = true;
    usleep(CAP_RATE * 1000);
  }
}

void PrintBuildInfo()
{
    FC2Version fc2Version;
    Utilities::GetLibraryVersion( &fc2Version );
    char version[128];
    sprintf( 
        version, 
        "FlyCapture2 library version: %d.%d.%d.%d\n", 
        fc2Version.major, fc2Version.minor, fc2Version.type, fc2Version.build );

    printf( "%s", version );

    char timeStamp[512];
    sprintf( timeStamp, "Application build date: %s %s\n\n", __DATE__, __TIME__ );

    printf( "%s", timeStamp );
}

void PrintCameraInfo( CameraInfo* pCamInfo )
{
    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime );
}

void PrintError( Error error )
{
    error.PrintErrorTrace();
}

bool CheckSoftwareTriggerPresence( Camera* pCam )
{
	const unsigned int k_triggerInq = 0x530;

	Error error;
	unsigned int regVal = 0;

	error = pCam->ReadRegister( k_triggerInq, &regVal );

	if (error != PGRERROR_OK)
	{
		PrintError( error );
		return false;
	}

	if( ( regVal & 0x10000 ) != 0x10000 )
	{
		return false;
	}

	return true;
}

bool PollForTriggerReady( Camera* pCam )
{
    const unsigned int k_softwareTrigger = 0x62C;
    Error error;
    unsigned int regVal = 0;

    do 
    {
        error = pCam->ReadRegister( k_softwareTrigger, &regVal );
        if (error != PGRERROR_OK)
        {
            PrintError( error );
			return false;
        }

    } while ( (regVal >> 31) != 0 );

	return true;
}

bool FireSoftwareTrigger( Camera* pCam )
{
    const unsigned int k_softwareTrigger = 0x62C;
    const unsigned int k_fireVal = 0x80000000;
    Error error;    

    error = pCam->WriteRegister( k_softwareTrigger, k_fireVal );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return false;
    }

    return true;
}



int RunSingleCamera(PGRGuid guid, int id)
{
    Error error;
    Camera cam;
    //GigECamera cam;
    
    

    printf( "Connecting to camera...\n" );

    // Connect to a camera
    error = cam.Connect(&guid);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    const unsigned int k_cameraPower = 0x610;
    const unsigned int k_powerVal = 0x80000000;
    error  = cam.WriteRegister( k_cameraPower, k_powerVal );
    if (error != PGRERROR_OK)
    {
      PrintError( error );
      return -1;
    }

    const unsigned int millisecondsToSleep = 100;
    unsigned int regVal = 0;
    unsigned int retries = 10;

    // Wait for camera to complete power-up
    do 
    { 
      usleep(millisecondsToSleep * 1000);
      error = cam.ReadRegister(k_cameraPower, &regVal);
      if (error == PGRERROR_TIMEOUT)
      {
      // ignore timeout errors, camera may not be responding to
      // register reads during power-up
      }
      else if (error != PGRERROR_OK)
      {
	PrintError( error );
	return -1;
      }

      retries--;
    } while ((regVal & k_powerVal) == 0 && retries > 0);
    // Check for timeout errors after retrying
    if (error == PGRERROR_TIMEOUT)
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
     PrintCameraInfo(&camInfo); 
//  
//    unsigned int numStreamChannels = 0;
//     error = cam.GetNumStreamChannels( &numStreamChannels );
//     if (error != PGRERROR_OK)
//     {
//         PrintError( error );
//         return -1;
//     }
// 
//     for (unsigned int i=0; i < numStreamChannels; i++)
//     {
//         GigEStreamChannel streamChannel;
//         error = cam.GetGigEStreamChannelInfo( i, &streamChannel );
//         if (error != PGRERROR_OK)
//         {
//             PrintError( error );
//             return -1;
//         }
// 
//         printf( "\nPrinting stream channel information for channel %u:\n", i );
//         //PrintStreamChannelInfo( &streamChannel );
//     }    
// 
//     printf( "Querying GigE image setting information...\n" );
// 
//     GigEImageSettingsInfo imageSettingsInfo;
//     error = cam.GetGigEImageSettingsInfo( &imageSettingsInfo );
//     if (error != PGRERROR_OK)
//     {
//         PrintError( error );
//         return -1;
//     }
// 
//     GigEImageSettings imageSettings;
//     imageSettings.offsetX = 0;
//     imageSettings.offsetY = 0;
//     imageSettings.height = imageSettingsInfo.maxHeight;
//     imageSettings.width = imageSettingsInfo.maxWidth;
//     imageSettings.pixelFormat = PIXEL_FORMAT_RAW8;
// 
//     printf( "Setting GigE image settings...\n" );
// 
//     error = cam.SetGigEImageSettings( &imageSettings );
//     if (error != PGRERROR_OK)
//     {
//         PrintError( error );
//         return -1;
//     }   
     
#ifndef SOFTWARE_TRIGGER_CAMERA
    // Check for external trigger support
    TriggerModeInfo triggerModeInfo;
    error = cam.GetTriggerModeInfo( &triggerModeInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    if ( triggerModeInfo.present != true )
    {
        printf( "Camera does not support external trigger! Exiting...\n" );
        return -1;
    }
#endif
     
     // Get current trigger settings
    TriggerMode triggerMode;
    error = cam.GetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

    // Set camera to trigger mode 0
    triggerMode.onOff = true;
    triggerMode.mode = 0;
    triggerMode.parameter = 0;

#ifdef SOFTWARE_TRIGGER_CAMERA
    // A source of 7 means software trigger
    triggerMode.source = 7;
#else
    // Triggering the camera externally using source 0.
    triggerMode.source = 0;
#endif
    
    error = cam.SetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    
    // Poll to ensure camera is ready
    bool retVal = PollForTriggerReady( &cam );
	if( !retVal )
	{
		printf("\nError polling for trigger ready!\n");
		return -1;
	}

    // Get the camera configuration
    FC2Config config;
    error = cam.GetConfiguration( &config );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    } 
    
    // Set the grab timeout to 5 seconds
    config.grabTimeout = 5000;

    // Set the camera configuration
    error = cam.SetConfiguration( &config );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    } 

    // Camera is ready, start capturing images
    printf( "Starting image capture...\n" );

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
#ifdef SOFTWARE_TRIGGER_CAMERA
	if (!CheckSoftwareTriggerPresence( &cam ))
	{
		printf( "SOFT_ASYNC_TRIGGER not implemented on this camera!  Stopping application\n");
		return -1;
	}
#else	
	printf( "Trigger the camera by sending a trigger pulse to GPIO%d.\n", 
      triggerMode.source );
#endif

    
    Image rawImage;  
    int seq = 0;
    while(1)
    { 
      // Check that the trigger is ready
#ifdef SOFTWARE_TRIGGER_CAMERA
      PollForTriggerReady( &cam);
      while(!(firecam[id])) usleep(SLEEP_MICRO_SEC * 1000);
      bool retVal = FireSoftwareTrigger( &cam );
        if ( !retVal )
        {
	    printf("\nError firing software trigger!\n");
	    return -1;        
	}
#endif
        // Grab image  
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
	
	firecam[id] = false;
    }         

    printf( "Stopping capture...\n" );

 triggerMode.onOff = false;    
    error = cam.SetTriggerMode( &triggerMode );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }
    printf( "\nFinished grabbing images\n" );

    // Stop capturing images
    error = cam.StopCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }      

    // Turn off trigger mode
    triggerMode.onOff = false;
    error = cam.SetTriggerMode( &triggerMode );
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

    boost::thread t(fireThread,numCameras);
    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
