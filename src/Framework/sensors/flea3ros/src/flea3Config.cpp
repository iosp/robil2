// FLEA 3 ROS 		By:Roman Kudinov

//
#ifdef LINUX

#include <unistd.h>

#endif
#include <boost/thread/thread.hpp>
#include "FlyCapture2.h"
#include <vector>
#include <ctime>
using namespace FlyCapture2;

PGRGuid cameraMasterSlave[2];
// image_transport::Publisher publishers[10];
void PrintCameraInfo( CameraInfo* pCamInfo )
{
    char macAddress[64];
    sprintf( 
        macAddress, 
        "%02X:%02X:%02X:%02X:%02X:%02X", 
        pCamInfo->macAddress.octets[0],
        pCamInfo->macAddress.octets[1],
        pCamInfo->macAddress.octets[2],
        pCamInfo->macAddress.octets[3],
        pCamInfo->macAddress.octets[4],
        pCamInfo->macAddress.octets[5]);

    char ipAddress[32];
    sprintf( 
        ipAddress, 
        "%u.%u.%u.%u", 
        pCamInfo->ipAddress.octets[0],
        pCamInfo->ipAddress.octets[1],
        pCamInfo->ipAddress.octets[2],
        pCamInfo->ipAddress.octets[3]);

    char subnetMask[32];
    sprintf( 
        subnetMask, 
        "%u.%u.%u.%u", 
        pCamInfo->subnetMask.octets[0],
        pCamInfo->subnetMask.octets[1],
        pCamInfo->subnetMask.octets[2],
        pCamInfo->subnetMask.octets[3]);

    char defaultGateway[32];
    sprintf( 
        defaultGateway, 
        "%u.%u.%u.%u", 
        pCamInfo->defaultGateway.octets[0],
        pCamInfo->defaultGateway.octets[1],
        pCamInfo->defaultGateway.octets[2],
        pCamInfo->defaultGateway.octets[3]);

    printf(
        "\n*** CAMERA INFORMATION ***\n"
        "Serial number - %u\n"
        "Camera model - %s\n"
        "Camera vendor - %s\n"
        "Sensor - %s\n"
        "Resolution - %s\n"
        "Firmware version - %s\n"
        "Firmware build time - %s\n"
        "GigE version - %u.%u\n"
        "User defined name - %s\n"
        "XML URL 1 - %s\n"
        "XML URL 2 - %s\n"
        "MAC address - %s\n"
        "IP address - %s\n"
        "Subnet mask - %s\n"
        "Default gateway - %s\n\n",
        pCamInfo->serialNumber,
        pCamInfo->modelName,
        pCamInfo->vendorName,
        pCamInfo->sensorInfo,
        pCamInfo->sensorResolution,
        pCamInfo->firmwareVersion,
        pCamInfo->firmwareBuildTime,
        pCamInfo->gigEMajorVersion,
        pCamInfo->gigEMinorVersion,
        pCamInfo->userDefinedName,
        pCamInfo->xmlURL1,
        pCamInfo->xmlURL2,
        macAddress,
        ipAddress,
        subnetMask,
        defaultGateway );
}


void PrintError( Error error )
{
    error.PrintErrorTrace();
}


void printRegister(GigECamera *cam,unsigned int reg)
{  
  unsigned int regVal = 0;
  Error error;
  error = cam->ReadRegister( reg, &regVal );
  if (error != PGRERROR_OK)
  {
      PrintError( error );
      return;
  }
  else
    printf("%x  %x\n",reg,regVal);
}

void setPWM(GigECamera *cam)
{
  CameraInfo camInfo;
   Error error;
    error = cam->GetCameraInfo(&camInfo);
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return;
    }
// 
  //  PrintCameraInfo(&camInfo);  
  printf("Setting %u camera's PWM\n", camInfo.serialNumber);
  
  cam->WriteRegister(0x1130,0x8004ff01);
  cam->WriteRegister(0x1134,0x5e005e00);

}

int main(int argc, char** argv)
{    
    
    Error error;
    BusManager busMgr;
    CameraInfo camInfo[10];
    unsigned int numCamInfo = 10;
    std::vector<GigECamera*> cams;
    GigECamera cam1;

    GigECamera cam2;

    cams.push_back(&cam1);

    cams.push_back(&cam2);

    TriggerMode triggerMode;
    
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
	
        
	PGRGuid guid;
        error = busMgr.GetCameraFromIndex(i, &guid);
        if (error != PGRERROR_OK)
        {
            PrintError( error );
            return -1;
        }

        error = cams[i]->Connect(&guid);
        if (error != PGRERROR_OK)
	{
	      PrintError( error );
	      return -1;
	}
	const unsigned int k_cameraPower = 0x610;
	const unsigned int k_powerVal = 0x80000000;
	error  = cams[i]->WriteRegister( k_cameraPower, k_powerVal );
	const unsigned int millisecondsToSleep = 100;
	unsigned int regVal = 0;
	unsigned int retries = 10;
	do 

	{

		usleep(millisecondsToSleep * 1000);

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
	if(camInfo.serialNumber == 13220212) setPWM(cams[i]);
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
		
		triggerMode.source=3;
		
		error = cams[i]->SetTriggerMode( &triggerMode );
		
		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}
		
		
		//Declare a Property struct.
		Property prop;
		//Define the property to adjust.
		prop.type = SHUTTER;
		//Ensure the property is on.
		prop.onOff = true;
		//Ensure auto-adjust mode is off.
		prop.autoManualMode = false;
		//Ensure the property is set up to use absolute value control.
		prop.absControl = true;
		//Set the absolute value of shutter to 20 ms.
		prop.absValue = 20;
		//Set the property.
		error = cams[i]->SetProperty( &prop );
		if (error != PGRERROR_OK)

		{

			PrintError( error );

			return -1;

		}		
    }

    
    return 0;
}
