// FLEA 3 ROS 		By:Roman Kudinov

//

#include "FlyCapture2.h"
// #include <ros/ros.h>
// #include <ros/spinner.h>
// #include <sensor_msgs/image_encodings.h>
// #include <sensor_msgs/Image.h>
// #include <sensor_msgs/CompressedImage.h>
// #include <cv_bridge/cv_bridge.h>
// #include <boost/thread/thread.hpp>
// #include <opencv2/opencv.hpp>
// #include <cv_bridge/cv_bridge.h>

// #include <image_transport/image_transport.h>


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

void setCamMaster(GigECamera *cam)
{
  CameraInfo camInfo;
   Error error;
    error = cam->GetCameraInfo(&camInfo);
//     if (error != PGRERROR_OK)
//     {
//         PrintError( error );
//         return -1;
//     }
// 
    PrintCameraInfo(&camInfo);  
  printf("Setting camera as master\n");
  printf("before registery setup\n");
  printRegister(cam,0x11f8);printRegister(cam,0x1104);printRegister(cam,0x1508);printRegister(cam,0x083c);printRegister(cam,0x081c);
  cam->WriteRegister(0x11f8,0x60000000);
  cam->WriteRegister(0x1104,0x40000030);
  cam->WriteRegister(0x1508,0x83000400);
  cam->WriteRegister(0x083c,0x80000200);
  //1100 0010 0000 0000 0000  
  cam->WriteRegister(0x081c,0xc200078d);
  printf("after registery setup\n");
  printRegister(cam,0x11f8);printRegister(cam,0x1104);printRegister(cam,0x1508);printRegister(cam,0x083c);printRegister(cam,0x081c);
  printRegister(cam,0x0604);printRegister(cam,0x0608);

}

void setCamSlave(GigECamera *cam)
{
  printf("Setting camera as slave\n");
  printf("before registery setup\n");
  cam->WriteRegister(0x11f8,0x40000000);
  cam->WriteRegister(0x0830,0x835e0000);
  cam->WriteRegister(0x0834,0xc2000021);
  cam->WriteRegister(0x081c,0xc200078d);
  printf("after registery setup\n");

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

    printf( "Querying GigE image setting information...\n" );

    GigEImageSettingsInfo imageSettingsInfo;
    error = cam.GetGigEImageSettingsInfo( &imageSettingsInfo );
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }

//     GigEImageSettings imageSettings;
//     imageSettings.offsetX = 0;
//     imageSettings.offsetY = 0;
//     imageSettings.height = imageSettingsInfo.maxHeight;
//     imageSettings.width = imageSettingsInfo.maxWidth;
//     imageSettings.pixelFormat = PIXEL_FORMAT_RGB;
// 
//     printf( "Setting GigE image settings...\n" );
// 
//     error = cam.SetGigEImageSettings( &imageSettings );
//     if (error != PGRERROR_OK)
//     {
//         PrintError( error );
//         return -1;
//     }
    if (cameraMasterSlave[0] == guid)
      setCamMaster(&cam);
    else if (cameraMasterSlave[1] == guid)
      setCamSlave(&cam);

    // Start capturing images
    error = cam.StartCapture();
    if (error != PGRERROR_OK)
    {
        PrintError( error );
        return -1;
    }


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
        if (i < 2)
	  cameraMasterSlave[i] = guid;
        if ( interfaceType == INTERFACE_GIGE )
	   RunSingleCamera(guid, i);
	
        
    }

    
    return 0;
}
