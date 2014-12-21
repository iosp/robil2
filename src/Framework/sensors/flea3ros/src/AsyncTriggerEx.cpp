//=============================================================================

// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.

//

// This software is the confidential and proprietary information of Point

// Grey Research, Inc. ("Confidential Information").  You shall not

// disclose such Confidential Information and shall use it only in

// accordance with the terms of the license agreement you entered into

// with PGR.

//

// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE

// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE

// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR

// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES

// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING

// THIS SOFTWARE OR ITS DERIVATIVES.

//=============================================================================

//=============================================================================

// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $

//=============================================================================



//#include "stdafx.h"

#ifdef LINUX

#include <unistd.h>

#endif

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
#include "FlyCapture2.h"

#include <ctime>

#include <vector>

//#include <process.h>

//#include <conio.h>



using namespace FlyCapture2;

image_transport::Publisher publishers[2];





bool endCode=false;

bool captureStopped=false;



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



bool CheckSoftwareTriggerPresence( GigECamera* pCam )

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



bool PollForTriggerReady( GigECamera* pCam )

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



bool FireSoftwareTrigger( GigECamera*pCam )

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



//Thread for triggering the camera

void TriggeringThread(std::vector<GigECamera*>* pParams)

{

	std::vector<GigECamera*> *cameras;

	int size = (pParams)->size();

	cameras = pParams;

	//cameras = &((std::vector<GigECamera*>*)pParams);

	GigECamera *masterCamera = cameras->at(0);

	GigECamera *slaveCamera = cameras->at(1);



	Error error;		



	

	while(!endCode)

	{

		//Checking both camera for trigger ready state

		

		if(PollForTriggerReady(masterCamera) && PollForTriggerReady(slaveCamera))

		{

			//Software triggering the camera


			printf("Firing\n");
			bool retVal = FireSoftwareTrigger( masterCamera );
			bool retVal1 = FireSoftwareTrigger( slaveCamera );
			if ( !retVal )

			{

				printf("\nError firing software trigger!\n");      

			}
			if ( !retVal1 )

			{

				printf("\nError firing software trigger!\n");      

			}



		}
		else
		  printf("no poll from slave\n");

	}
	printf("exiting firing thread\n");



}



void RetrieveBufferThread(GigECamera* pParams,int id)

{

	GigECamera*cam= pParams;

	Error error;

	Image rawImage;
	int seq = 0;


	while(!endCode)

	{

		error = cam->RetrieveBuffer( &rawImage );

		if (error != PGRERROR_OK)

		{
		  printf("error in cam %d\n",id);

			PrintError( error );   
			continue;

		}
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
		printf("publish cam %d\n",id);
		seq++;

	}





	error=cam->StopCapture();

	if (error != PGRERROR_OK)

	{

		PrintError( error );            

	}

	error = cam->Disconnect();

	if (error != PGRERROR_OK)

	{

		PrintError( error );            

	}
	printf("exiting poll thread %d\n",id);
}



int main(int argc, char** argv)

{ 

	PrintBuildInfo();
	ros::init(argc, argv, "flea3ros");
	ros::NodeHandle n;
	image_transport::ImageTransport it(n);



	Error error;



	BusManager busMgr;

	unsigned int numCameras;

	error = busMgr.GetNumOfCameras(&numCameras);

	if (error != PGRERROR_OK)

	{

		PrintError( error );

		return -1;

	}



	printf( "Number of cameras detected: %u\n", numCameras );



	if ( numCameras < 1 )

	{

		printf( "Insufficient number of cameras... exiting\n" );

		return -1;

	}



	std::vector<GigECamera*> cams;

	GigECamera cam1;

	GigECamera cam2;

	cams.push_back(&cam1);

	cams.push_back(&cam2);



	TriggerMode triggerMode;

	StrobeControl mStrobe;



	PGRGuid guid;



	for(unsigned int i=0; i<numCameras;i++)

	{



		error = busMgr.GetCameraFromIndex(i, &guid);

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;
//=============================================================================

// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.

//

// This software is the confidential and proprietary information of Point

// Grey Research, Inc. ("Confidential Information").  You shall not

// disclose such Confidential Information and shall use it only in

// accordance with the terms of the license agreement you entered into

// with PGR.

//

// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE

// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE

// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR

// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES

// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING

// THIS SOFTWARE OR ITS DERIVATIVES.

//=============================================================================

//=============================================================================

// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $

//=============================================================================

		}





		// Connect to a camera

		error = cams[i]->Connect(&guid);

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		// Power on the camera

		const unsigned int k_cameraPower = 0x610;

		const unsigned int k_powerVal = 0x80000000;

		error  = cams[i]->WriteRegister( k_cameraPower, k_powerVal );

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

#if defined(WIN32) || defined(WIN64)

			Sleep(millisecondsToSleep);    

#else

			usleep(millisecondsToSleep * 1000);

#endif

			error = cams[i]->ReadRegister(k_cameraPower, &regVal);

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



		//error=cams[i]->RestoreFromMemoryChannel(0);

		// Get the camera information

		CameraInfo camInfo;

		error = cams[i]->GetCameraInfo(&camInfo);

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		PrintCameraInfo(&camInfo);      



		GigEStreamChannel streamChannel;

		streamChannel.packetSize=9000;

		GigEImageSettingsInfo imageSettingsInfo;
		error = cams[i]->GetGigEImageSettingsInfo( &imageSettingsInfo );
		if (error != PGRERROR_OK)
		{
		    PrintError( error );
		    return -1;
		}

		GigEImageSettings imageSettings;



		imageSettings.height=imageSettingsInfo.maxHeight;;

		imageSettings.width=imageSettingsInfo.maxWidth;

		imageSettings.pixelFormat = FlyCapture2::PIXEL_FORMAT_RAW8;



		error= cams[i]->SetGigEImageSettings(&imageSettings);

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		printf("Image Resolution: %d x %d",imageSettings.width,imageSettings.height);





		// Get current trigger settings



		error = cams[i]->GetTriggerMode( &triggerMode );

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		TriggerDelay tDelay;

		error=cams[i]->GetTriggerDelay(&tDelay);

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		// Set camera to trigger mode 0

		triggerMode.onOff = true;

		triggerMode.mode = 14;

		triggerMode.parameter = 0;

		





		/*mStrobe.delay = 0;

		mStrobe.duration = 0;*/

		mStrobe.onOff = true;

		mStrobe.polarity = 0;



		if(i==0)

		{

			triggerMode.source=7;

			triggerMode.polarity=0;

			mStrobe.source=1;

			

			//mStrobe.delay=3;

		}



		if(i==1)

		{

			triggerMode.source=0;

			triggerMode.polarity=0;

			mStrobe.source=2;



			// Setting trigger delay

			tDelay.onOff=true;

			tDelay.absControl=true;

			tDelay.absValue=0.001;



			//Set the start of strobe for slave camera to start of exposure

			const unsigned int k_strobeRegister = 0x1104;

			unsigned int k_regVal = 0x00000000;

			error=cams[i]->ReadRegister(k_strobeRegister,&k_regVal);

			error  = cams[1]->WriteRegister( k_strobeRegister, k_regVal||0x80000000);





			

			//mStrobe.duration=20;



			error=cams[i]->SetTriggerDelay(&tDelay);

			if (error != PGRERROR_OK)

			{

				PrintError( error );

				return -1;

			}

			//mStrobe.duration=20;

		}

		error = cams[i]->SetTriggerMode( &triggerMode );



		error=cams[i]->SetStrobe(&mStrobe);



		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



		// Poll to ensure camera is ready

		bool retVal = PollForTriggerReady( cams[i] );

		if( !retVal )

		{

			printf("\nError polling for trigger ready!\n");

			return -1;

		}





		// Get the camera configuration

		FC2Config config;

		error = cams[i]->GetConfiguration( &config );

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		} 



		// Set the grab timeout to 5 seconds

		config.grabTimeout = 5000;



		// Set the camera configuration

		error = cams[i]->SetConfiguration( &config );

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		} 
		char topicName[20];
		sprintf(topicName, "SENSORS/FLEA3/%d", i);
// 	publishers[i] = n.advertise<sensor_msgs::Image>(topicName, 10);
		publishers[i] = it.advertise(topicName, 10);

		error = cams[i]->StartCapture();

		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}



	}           



	boost::thread t(TriggeringThread,&cams);

	boost::thread t1(RetrieveBufferThread,cams.at(0),0);

	boost::thread t2(RetrieveBufferThread,cams.at(1),1);//=============================================================================

// Copyright © 2008 Point Grey Research, Inc. All Rights Reserved.

//

// This software is the confidential and proprietary information of Point

// Grey Research, Inc. ("Confidential Information").  You shall not

// disclose such Confidential Information and shall use it only in

// accordance with the terms of the license agreement you entered into

// with PGR.

//

// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE

// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE

// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR

// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES

// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING

// THIS SOFTWARE OR ITS DERIVATIVES.

//=============================================================================

//=============================================================================

// $Id: AsyncTriggerEx.cpp,v 1.21 2010-07-22 22:51:51 soowei Exp $

//=============================================================================

ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();


// 	while(!_kbhit())
// 
// 	{
// 
// 		Sleep(1000);
// 
// 	}



	endCode=true;
      printf("exiting\n");


	getchar();



	return 0;

}