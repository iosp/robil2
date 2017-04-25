#include <boost/bind.hpp>

#include <stdio.h>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/sensors/sensors.hh"
#include "gazebo/gazebo.hh"
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <ctime>

#include <math.h>
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"

#include <netdb.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <unistd.h>
#include <netinet/in.h>

#include "../include/IPON.h"

#include <boost/thread.hpp>
#include <atomic>

#include <boost/thread/mutex.hpp>

#pragma pack(1)

using namespace std;


//
// Real World : positive Azimuth angel is clockwise (from North to East, 0 is in the North) ==> azimuth = atan2(DY,DX)
//              X is the North, Y is the East, Z - directed downwards  
//        N (X)
//         ^
//         |
//  W ------------> E (Y)
//         |
//         |
//          S
//
//
// In GAZEBO :  z - is directed upwards, positive Azimuth angel is counterclockwise
//          x
//          ^
//          |
//  y <----------- 
//          |
//          |
//
//
// therefore :  
//	Azimuth_world = -theta_gazebo 
//	Pitch_world = -pitch_gazebo
//	Roll_world = roll_gazebo
//	bering = -atan2(Dy,Dx) , (for translation calculation)

//	x_gazebo direction represent the North,
//	y_gazebo direction represent the West,
//	z_gazebo direction represent the Upwards
//



namespace gazebo
{   
  class IPON : public ModelPlugin
  {

  public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      srand(time(NULL));

	  _seq = 0;
      this->_model = _parent;

      loadParametersFromSdf(_sdf);

      _gps_ant_link = _parent->GetLink(_gps_ant_link_name);
      _init_pos = _gps_ant_link->GetWorldCoGPose().pos;

      lastTimeOf100HZMSGSending = 0;
      lastTimeOf1HZMSGSending = 0;

      sensors::SensorPtr sensorIMU;
      sensorIMU = sensors::SensorManager::Instance()->GetSensor(SENSOR_IMU_NAME);
      if(!sensorIMU)
      {
		string error = "GPS/INS Sensor Model \"" + _parent->GetName() + "\" failed to locate some of his sub-sensors.\n(Do the names in the .sdf match the names in the .cpp?)\n";
		gzthrow(error);
		return;
      }
#if GAZEBO_MAJOR_VERSION >= 7
      _imu = std::dynamic_pointer_cast<sensors::ImuSensor>(sensorIMU);
#else
      _imu = boost::dynamic_pointer_cast<sensors::ImuSensor>(sensorIMU);
#endif
	  UDPSocketIsConnected = initUDPConnection();
	  initTCPConnection();

      _udpSenderThread=boost::thread(&IPON::sendThreadMethod,this);

      // Listen to the update event. This event is broadcast every simulation iteration.
      this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&IPON::OnUpdate, this, _1));


      vecToSend = new (std::vector<std::pair<common::Time, char[1000]> *>);

      FC_LastTime_Second = 0;
      FC_sumOfFrequencyForAverage = 0;
      FC_counterOfSecondsForFrequencyAverage = 0;
      FC_counterOfMsgInSec = 0;

    }


	void loadParametersFromSdf(sdf::ElementPtr _sdf)
	{
	      _start_latitude=_start_longitude=0;
	      _frequency=100;
	      Delay = 0.00;
	      IP = "127.0.0.1";
	      //load config from sdf
	      if (_sdf->HasElement("start_latitude"))
	    	  _sdf->GetElement("start_latitude")->GetValue()->Get(_start_latitude);
	      if (_sdf->HasElement("start_longitude"))
	    	  _sdf->GetElement("start_longitude")->GetValue()->Get(_start_longitude);
	      if (_sdf->HasElement("frequency"))
	    	  _sdf->GetElement("frequency")->GetValue()->Get(_frequency);
	      if (_sdf->HasElement("gps_ant_link_name"))
	    	  _sdf->GetElement("gps_ant_link_name")->GetValue()->Get<std::string>(_gps_ant_link_name);
	      if (_sdf->HasElement("imu_unit_name"))
	    	  _sdf->GetElement("imu_unit_name")->GetValue()->Get<std::string>(_imu_unit_name);
	      if (_sdf->HasElement("Delay"))
	    	  _sdf->GetElement("Delay")->GetValue()->Get(Delay);
	      if (_sdf->HasElement("IP"))
	    	  _sdf->GetElement("IP")->GetValue()->Get<std::string>(IP);

	    _gps_noise=_rp_noise=_yaw_noise=_gy_noise=_acc_noise=_acc_bias=_gy_bias=_spd_noise=0;
	    if(_sdf->HasElement("noise"))
	    	{
				sdf::ElementPtr elem = _sdf->GetElement("noise");
				if (elem->HasElement("gps"))
				  elem->GetElement("gps")->GetValue()->Get(_gps_noise);
				cout << "GPS noise=: " << _gps_noise << endl;
				if (elem->HasElement("gps_speed"))
				  elem->GetElement("gps_speed")->GetValue()->Get(_spd_noise);
				if (elem->HasElement("rollpitch"))
				  elem->GetElement("rollpitch")->GetValue()->Get(_rp_noise);
				if (elem->HasElement("yaw"))
				  elem->GetElement("yaw")->GetValue()->Get(_yaw_noise);
				if (elem->HasElement("acc_bias"))
				  elem->GetElement("acc_bias")->GetValue()->Get(_acc_bias);
				if (elem->HasElement("_gyro_bias"))
				  elem->GetElement("_gyro_bias")->GetValue()->Get(_gy_bias);
				if (elem->HasElement("acc_noise"))
				  elem->GetElement("acc_noise")->GetValue()->Get(_acc_noise);
				if (elem->HasElement("gyro_noise"))
				  elem->GetElement("gyro_noise")->GetValue()->Get(_gy_noise);
	      	  }
	}
    // Called by the world update start event
	void OnUpdate(const common::UpdateInfo & _info)
    {
      //manage frequency
	  sim_Time = _info.simTime;
      
      if(!UDPSocketIsConnected && !initUDPConnection()) // client is not connected and initConnection() is failed
      {
    	  return;
      }

      if(!TCPIsAlive())
      {
    	  initTCPConnection();
      }

      //100HZ msg
      double dtOf100HZ = sim_Time.Double() - lastTimeOf100HZMSGSending.Double();
   	  if( dtOf100HZ - (double)TIME_INTERVAL_100HZ >= -0.00001)
   	  {
       	  sendIMUToUDP_100HZMESSAGE(_info);
   		  lastTimeOf100HZMSGSending = _info.simTime;
   	  }
      //1HZ msg
      if(!shouldSend1HZMsg)
      {
    	  double dtOf1HZ = sim_Time.Double() - lastTimeOf1HZMSGSending.Double();
    	  if( dtOf1HZ - (double)TIME_INTERVAL_1HZ >= -0.00001 )
    	  {
    		  sendIMUToUDP_1HZMESSAGE(_info);
    		  shouldSend1HZMsg = true;
    		  lastTimeOf1HZMSGSending = _info.simTime;
    	  }
      }
    }

    gazebo::math::Vector3 getLanLonAlt_Velocities(){
    	gazebo::math::Vector3 vel=_gps_ant_link->GetWorldLinearVel();
    	return (vel);
    }

    LatLonAlt getLanLonAlt(){
		// GPS pose
		math::Pose pose=_gps_ant_link->GetWorldPose();
		gazebo::math::Vector3 pos = pose.pos;

		pos.x += add_gps_noise();
		pos.y += add_gps_noise();
		double dist = sqrt((pos.x-_init_pos.x)*(pos.x-_init_pos.x)+(pos.y-_init_pos.y)*(pos.y-_init_pos.y));
		double brng;
		if(!(pos.GetLength()*_init_pos.GetLength()))
		{
			brng = - atan2(pos.y,pos.x);
		}
		else
		{
			brng = - atan2((pos.y-_init_pos.y),(pos.x-_init_pos.x));
		}

		double R = 6378.1*1000;
		float b_lat = _start_latitude*PI/180;
		float b_lon = _start_longitude*PI/180;
		float d_ang = dist/R;
		float a = acos(cos(d_ang)*cos(PI/2-b_lat) + sin(PI/2-b_lat)*sin(d_ang)*cos(brng));
		float B = asin(sin(d_ang)*sin(brng)/sin(a));

		LatLonAlt result;
		result.latitude = 180/PI * (PI/2-a);
		result.longitude = 180/PI * (B + b_lon);
		result.altitude = pos.z;

		//ROS_INFO("latitude: %g, longitude: %g, altitude: %g", latitude, longitude, altitude);

		return result;
    }
    double sampleNormal()
    {
      double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
      double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
      double r = u * u + v * v;
      if (r == 0 || r > 1) return sampleNormal();
      double c = sqrt(-2 * log(r) / r);
      return u * c;
    }
    double add_gps_noise()
    {
      return sampleNormal()*_gps_noise;
    }

	void sendThreadMethod()
	  {
	   	while(true)
	   	{
	   		if(vecToSend && !vecToSend->empty())
	   		{
		   		vecToSend_mutex.lock();
    	   		std::pair<common::Time, char[1000]>* front = vecToSend->front();
		   		vecToSend_mutex.unlock();

   				double diff = ((*(front)).first.Double() + Delay) - sim_Time.Double();
	   			if(diff < 0.00001)
	   			{
	   			int bytes_sent=0;

	   			while(bytes_sent<sizeof(PHSPERIODIC100HZMESSAGE) && UDPSocketIsConnected)
	   			{
    				int n = sendto(UDPsockfd,(*(front)).second+bytes_sent, sizeof(PHSPERIODIC100HZMESSAGE)-bytes_sent, 0, (sockaddr *)&UDPserv_addr, sizeof(UDPserv_addr));
        	  		//ROS_INFO("IPON 100HZ msg send time : %f  ", (*(vecToSend->front())).first.Double());
	   				if(n>0)
					{
					   bytes_sent+=n;
					  //ROS_INFO("bytes_sent = %d", bytes_sent);
					}
	   				else if(n<0)
					{
					   if(UDPsockfd>0)
					   {
						   cout << "shutdown" << endl;
						   shutdown(UDPsockfd,SHUT_RDWR);
						   UDPsockfd=-1;
					   }
					   UDPSocketIsConnected=false;
					   ROS_INFO("NOT SEND, errno = %s", strerror(errno));
					   break;
					}

					if(bytes_sent>=sizeof(PHSPERIODIC100HZMESSAGE))
					{
				   		vecToSend_mutex.lock();
						std::pair<common::Time, char[1000]> *temp = vecToSend->front();
						vecToSend->erase( (vecToSend->begin()) );
						delete temp;
				   		vecToSend_mutex.unlock();

						// frequency checking - if average of frequency is not correct for 10 seconds warn message will appear
		      	  		if (sim_Time.sec - FC_LastTime_Second >= 1 )
		      	  		{
		      	  		   FC_counterOfMsgInSec++;
	      	  			   FC_LastTime_Second = sim_Time.sec;
	      	  			   FC_sumOfFrequencyForAverage += FC_counterOfMsgInSec;
	      	  			   FC_counterOfSecondsForFrequencyAverage++;

	      	  			   //ROS_INFO("IPON 100HZ msg/sec: %d \n\n", FC_counterOfMsgInSec);
	      	  			   FC_counterOfMsgInSec = 0;

	      	  			   if(FC_counterOfSecondsForFrequencyAverage == 10)
	      	  			   {
	      	  				  if((FC_sumOfFrequencyForAverage / FC_counterOfSecondsForFrequencyAverage) - (1 / (double)TIME_INTERVAL_100HZ) != 0)
	      	  					  ROS_WARN("IPON: error in frequency of 100HZ message, Frequency Average = %f and it's should be = %f", FC_sumOfFrequencyForAverage / FC_counterOfSecondsForFrequencyAverage, (double)1 / TIME_INTERVAL_100HZ);

		        	  			  FC_sumOfFrequencyForAverage = 0;
		        	  			  FC_counterOfSecondsForFrequencyAverage = 0;
		        	  		   }
		        	  	}
		      	  		else
		      	  		{
		      	  			FC_counterOfMsgInSec++;
		      	  		}
					}
			    }
	   		    }
    		}
	   		if(shouldSend1HZMsg)
	   		{
   		        int bytes_sent=0;
	   			while(bytes_sent<sizeof(PHSPERIODIC1HZMESSAGE) && UDPSocketIsConnected)
			   {
	   				//ROS_INFO("IPON 1HZ   before: %g", sim_Time.Double());
	   				int n = sendto(UDPsockfd,bufferOf1HZMsg+bytes_sent, sizeof(PHSPERIODIC1HZMESSAGE)-bytes_sent, 0, (sockaddr *)&UDPserv_addr, sizeof(UDPserv_addr));
	   				//ROS_INFO("IPON 1HZ   after : %g", sim_Time.Double());

	   				if(n>0)
					{
					   bytes_sent+=n;
					  //ROS_INFO("bytes_sent = %d", bytes_sent);
					}
					if(n<0)
					{
					   if(UDPsockfd>0)
					   {
						   cout << "shutdown" << endl;
						   shutdown(UDPsockfd,SHUT_RDWR);
						   UDPsockfd=-1;
					   }
					   UDPSocketIsConnected=false;
					   shouldSend1HZMsg=false;
					   ROS_INFO("NOT SEND, errno = %s", strerror(errno));
					   break;
					}
			   }
    			shouldSend1HZMsg=false;
    		}

    	}
    }

    bool initUDPConnection()
    {
        UDPsockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if(UDPsockfd<0)
        {
        	ROS_ERROR("IPON, function: initConnection(), IPON socket not connected\n");
      	  	return false;
        }
 		memset((char *) & UDPserv_addr, 0,sizeof(UDPserv_addr));
 		UDPPortno= atoi(UDP_PORT_IPON);
 		UDPserv_addr.sin_family=AF_INET;
 		UDPserv_addr.sin_port=htons(UDPPortno);
 		if(inet_aton(IP.c_str(), &UDPserv_addr.sin_addr)==0)
 		{
 			ROS_ERROR("IPON, function: initConnection(), IPON inet_aton() failed\n");
  	        return false;
  		}
 		UDPSocketIsConnected = true;
    	ROS_INFO("initialize of UDP connection success");
 		return true;
    }
    bool initTCPConnection()
    {
 		memset((char *) & TCPserv_addr, 0,sizeof(TCPserv_addr));
    	TCPsockfd = socket(AF_INET, SOCK_STREAM, 0);
    	TCPserv_addr.sin_family = AF_INET;
    	TCPserv_addr.sin_addr.s_addr = INADDR_ANY;
    	TCPserv_addr.sin_port = htons(TCP_PORT_IPON);

    	if(bind(TCPsockfd, (struct sockaddr *)&TCPserv_addr, sizeof(TCPserv_addr)) < 0)
    	{
    		ROS_INFO("initTCPConnection, bind failed");
    		return false;
    	}

    	listen(TCPsockfd, 5);
    	ROS_INFO("initialize of TCP connection success");
    	return true;
    }
    bool TCPIsAlive()
    {
    	int error = 0;
    	socklen_t len = sizeof(error);
    	int retval = getsockopt(TCPsockfd, SOL_SOCKET, SO_ERROR, &error, &len);

    	if(retval != 0)
    	{
    		ROS_INFO("TCPIsAlive: error getting socket error code, errno: %s", strerror(retval));
    		return false;
    	}
    	if(error != 0)
    	{
    		ROS_INFO("TCPIsAlive: socket error, errno: %s", strerror(error));
    		return false;
    	}
    	return true;
    }

    template<class T1, class T2>
    void insertBetweenRanges(T1& dest, T2 src, double downLimit, double upLimit)
    {
      if(src < downLimit)
        dest = downLimit;
      else if(src > upLimit)
        dest = upLimit;
      else
        dest = src;
    }

    void sendIMUToUDP_1HZMESSAGE(const common::UpdateInfo & _info)
	{
		//ATTENTION: if you change any of the constants values in this function, you must use the 'insertBetweenRanges' function according to the ICD of this message.
		//example you can find in 'sendIMUToUDP_100HZMESSAGE' function
		PHSPERIODIC1HZMESSAGE IPON_MsgToSend;

		IPON_MsgToSend.SOM														= 42405; 		// defiantly not used in SAHAR (value is the default by the ICD)
		IPON_MsgToSend.Message_ID_Accepted_From_EGI								= E_MESSAGE_ID_ACCEPTED_FROM_EGI::E_MESSAGE_ID_ACCEPTED_FROM_EGI_PERIODIC_1HZ; // ID for 1Hz message
		IPON_MsgToSend.blockLength												= 60;
		IPON_MsgToSend.INS_Time_Of_Nav_Data										= ((time(NULL) - 315964800) % (7*24*60*60));
		IPON_MsgToSend.GPS_TINE_EGI												= _info.realTime.Double();
		IPON_MsgToSend.INS_DISTANCE_TRAVELED									= 0;	// defiantly not used in SAHAR
		LatLonAlt currentLatLonAlt = getLanLonAlt();
		IPON_MsgToSend.GPS_LAT_Egi												= currentLatLonAlt.latitude;	// defiantly not used in SAHAR
		IPON_MsgToSend.GPS_LONG_Egi												= currentLatLonAlt.longitude;	// defiantly not used in SAHAR
		insertBetweenRanges<double, double>(IPON_MsgToSend.GPS_Altitude_Egi, currentLatLonAlt.altitude, -16384, 16383); // defiantly not used in SAHAR
		IPON_MsgToSend.Alignment_Countdown										= 0;

		IPON_MsgToSend.Status_word.s1_status.INS_in_Exclusive_ZUPT_mode			= 0;
		IPON_MsgToSend.Status_word.s1_status.INS_in_startup_mode				= 0;
		IPON_MsgToSend.Status_word.s1_status.INS_in_survey_mode					= 1;
		IPON_MsgToSend.Status_word.s1_status.INS_normal_or_SH_or_move_align		= 0;
		IPON_MsgToSend.Status_word.s1_status.INS_ready_for_Align_on_the_move	= 0;
		IPON_MsgToSend.Status_word.s1_status.Position_update_request			= 0;
		IPON_MsgToSend.Status_word.s1_status.Startup_Complete					= 1;
		IPON_MsgToSend.Status_word.s1_status.Zero_velocity_stop_request			= 0;

		IPON_MsgToSend.Status_word.s2_status.Zero_Velocity_update_in_progress	= 1;
		IPON_MsgToSend.Status_word.s2_status.Travel_Lock_status					= 1;
		IPON_MsgToSend.Status_word.s2_status.NA_Egi								= 0;
		IPON_MsgToSend.Status_word.s2_status.NA1_Egi							= 0;
		IPON_MsgToSend.Status_word.s2_status.INS_or_ODO_or_GPS_malfunction		= 1;
		IPON_MsgToSend.Status_word.s2_status.INS_in_align_on_the_move			= 0;
		IPON_MsgToSend.Status_word.s2_status.INS_in_SH_align_mode				= 0;
		IPON_MsgToSend.Status_word.s2_status.INS_alert							= 0;

		IPON_MsgToSend.Status_word.s3_status.Degraded_Survey					= 0;
		IPON_MsgToSend.Status_word.s3_status.INS_in_motion						= 0;
		IPON_MsgToSend.Status_word.s3_status.NA1_Egi							= 0;
		IPON_MsgToSend.Status_word.s3_status.NA_Egi								= 0;
		IPON_MsgToSend.Status_word.s3_status.Odo_calibration_completed			= 0;
		IPON_MsgToSend.Status_word.s3_status.Odo_calibration_in_process			= 0;
		IPON_MsgToSend.Status_word.s3_status.Odo_damping_in_process				= 0;
		IPON_MsgToSend.Status_word.s3_status.Orientation_attitude_data_valid	= 1;

		IPON_MsgToSend.Status_word.s4_status.INS_SH_Shutdown_failed				= 0;
		IPON_MsgToSend.Status_word.s4_status.INS_Shutdown_complete_successful	= 0;
		IPON_MsgToSend.Status_word.s4_status.INS_in_standby_mode				= 0;
		IPON_MsgToSend.Status_word.s4_status.INS_wait_for_gps					= 0;
		IPON_MsgToSend.Status_word.s4_status.Integrated_mode_of_operation		= 1;
		IPON_MsgToSend.Status_word.s4_status.NA1_Egi							= 0;
		IPON_MsgToSend.Status_word.s4_status.NA_Egi								= 0;
		IPON_MsgToSend.Status_word.s4_status.SH_Shutdown_test_completed			= 0;
		IPON_MsgToSend.Alert_word_1.Alert_D2;										// defiantly not used in SAHAR
		IPON_MsgToSend.Alert_word_2.Alert_D3;										// defiantly not used in SAHAR
		IPON_MsgToSend.Alert_word_2.Alert_D4;										// defiantly not used in SAHAR
		IPON_MsgToSend.Alert_word_3.Alert_D5;										// defiantly not used in SAHAR
		IPON_MsgToSend.Alert_word_3.Alert_D6;										// defiantly not used in SAHAR
		IPON_MsgToSend.Azimuth_Error_RMS										= 26;										// value by the real IPON
		IPON_MsgToSend.Velocity_error_RMS										= 0.0099999998;
		IPON_MsgToSend.INS_Horizontal_Position_Error							= 1;
		IPON_MsgToSend.INS_Altitude_Error										= 1;
		IPON_MsgToSend.Roll_Error_RMS											= 0.1;
		IPON_MsgToSend.Pitch_Error_RMS											= 0.1;
		IPON_MsgToSend.Number_Of_Satellites										= 0;
		IPON_MsgToSend.Figure_Of_Merit											= E_FOM::E_FOM_SMALLER_THAN_25M;			// position error < 25m (1 sigma)
		IPON_MsgToSend.week														= (time(NULL) - 315964800) / (7*24*60*60);							// computer time, not UTM (the real GPS time is UTM).
		IPON_MsgToSend.GPS_UTC_offset											= 17;
		bzero(&IPON_MsgToSend.Bytes_2_reserved, 2);									// defiantly not used in SAHAR
		IPON_MsgToSend.Checksum_Egi												= 0;// defiantly not used in SAHAR

    	bzero(bufferOf1HZMsg, 1000);
    	unsigned char* pointToBuffer = bufferOf1HZMsg;
    	char* pointToSrc;
    	double dTemp = 0;
    	int iTemp = 0;
    	unsigned int uiTemp = 0;
    	short shTemp = 0;
    	unsigned short ushTemp = 0;
    	unsigned char uchTemp = 0;
    	char chTemp = '\0';

    	//SUM
    	memcpy(pointToBuffer, &IPON_MsgToSend.SOM, 2);
    	pointToBuffer+=2;

    	//Message_ID_Accepted_From_EGI
    	iTemp = littleEndianToBig<E_MESSAGE_ID_ACCEPTED_FROM_EGI>(IPON_MsgToSend.Message_ID_Accepted_From_EGI);
    	pointToSrc = (char*)&iTemp;
    	pointToSrc+=3;
    	memcpy(pointToBuffer, pointToSrc, 1);
    	pointToBuffer+=1;
    	pointToSrc = NULL;

    	//blockLength
    	ushTemp = IPON_MsgToSend.blockLength;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	pointToSrc = (char*)&ushTemp;
    	pointToSrc+=1;
    	memcpy(pointToBuffer, pointToSrc, 1);
    	pointToBuffer+=1;
    	pointToSrc = NULL;

    	//INS_Time_Of_Nav_Data
    	uiTemp = IPON_MsgToSend.INS_Time_Of_Nav_Data / ((double)((((1048576.0) - (0.0)) / ((4294967295.0) - (0.0)))));
    	uiTemp = littleEndianToBig<unsigned int>(uiTemp);
    	memcpy(pointToBuffer, (char*)&uiTemp, 4);
    	pointToBuffer+=4;

    	//GPS_TINE_EGI
    	iTemp = IPON_MsgToSend.GPS_TINE_EGI;
    	iTemp = littleEndianToBig<int>(iTemp);
    	memcpy(pointToBuffer, (char*)&iTemp, 4);
    	pointToBuffer+=4;

    	//INS_DISTANCE_TRAVELED
    	uiTemp = IPON_MsgToSend.INS_DISTANCE_TRAVELED;
    	uiTemp = littleEndianToBig<unsigned int>(uiTemp);
    	memcpy(pointToBuffer, (char*)&uiTemp, 4);
    	pointToBuffer+=4;

    	//GPS_LAT_Egi
    	dTemp = (IPON_MsgToSend.GPS_LAT_Egi - ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))))) / ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483647.0)))));
    	iTemp = dTemp;
    	iTemp = littleEndianToBig<int>(iTemp);
    	memcpy(pointToBuffer, (char*)&iTemp, 4);
    	pointToBuffer+=4;

    	//GPS_LONG_Egi
    	dTemp = (IPON_MsgToSend.GPS_LONG_Egi - ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))))) / ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483647.0)))));
    	iTemp = dTemp;
    	iTemp = littleEndianToBig<int>(iTemp);
    	memcpy(pointToBuffer, (char*)&iTemp, 4);
    	pointToBuffer+=4;

    	//GPS_Altitude_Egi
    	shTemp = IPON_MsgToSend.GPS_Altitude_Egi;
    	shTemp = littleEndianToBig<short>(shTemp);
    	memcpy(pointToBuffer, (char*)&shTemp, 2);
    	pointToBuffer+=2;

    	//Alignment_Countdown
    	ushTemp = IPON_MsgToSend.Alignment_Countdown;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//structs
        chTemp |= IPON_MsgToSend.Status_word.s2_status.Zero_Velocity_update_in_progress << 7;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.NA_Egi << 6;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.INS_or_ODO_or_GPS_malfunction << 5;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.Travel_Lock_status << 4;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.NA1_Egi << 3;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.INS_in_SH_align_mode << 2;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.INS_alert << 1;
        chTemp |= IPON_MsgToSend.Status_word.s2_status.INS_in_align_on_the_move << 0;
    	chTemp = IPON_MsgToSend.Figure_Of_Merit;
    	memcpy(pointToBuffer, (char*)&chTemp, 1);
    	pointToBuffer+=1;

    	chTemp |= IPON_MsgToSend.Status_word.s1_status.INS_in_startup_mode << 7;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.Startup_Complete << 6;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.INS_normal_or_SH_or_move_align << 5;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.INS_in_survey_mode << 4;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.INS_in_Exclusive_ZUPT_mode << 3;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.Zero_velocity_stop_request << 2;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.Position_update_request << 1;
    	chTemp |= IPON_MsgToSend.Status_word.s1_status.INS_ready_for_Align_on_the_move << 0;
    	memcpy(pointToBuffer, (char*)&chTemp, 1);
    	pointToBuffer+=1;

    	chTemp |= IPON_MsgToSend.Status_word.s4_status.SH_Shutdown_test_completed << 7;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.Integrated_mode_of_operation << 6;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.INS_SH_Shutdown_failed << 5;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.INS_Shutdown_complete_successful << 4;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.NA_Egi << 3;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.INS_in_standby_mode << 2;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.INS_wait_for_gps << 1;
    	chTemp |= IPON_MsgToSend.Status_word.s4_status.NA1_Egi << 0;
    	memcpy(pointToBuffer, (char*)&chTemp, 1);
    	pointToBuffer+=1;

    	chTemp |= IPON_MsgToSend.Status_word.s3_status.Odo_calibration_in_process << 7;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.Odo_damping_in_process << 6;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.Odo_calibration_completed << 5;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.NA_Egi << 4;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.NA1_Egi << 3;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.INS_in_motion << 2;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.Orientation_attitude_data_valid << 1;
    	chTemp |= IPON_MsgToSend.Status_word.s3_status.Degraded_Survey << 0;
    	memcpy(pointToBuffer, (char*)&chTemp, 1);
    	pointToBuffer+=1;


    	//Alert Structures
    	pointToBuffer+=8;


    	//Azimuth_Error_RMS
    	ushTemp = IPON_MsgToSend.Azimuth_Error_RMS;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Velocity_error_RMS
    	ushTemp = IPON_MsgToSend.Velocity_error_RMS / ((double) ((((200.0) - (0.0)) / ((20000.0) - (0.0)))));
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//INS_Horizontal_Position_Error
    	ushTemp = IPON_MsgToSend.INS_Horizontal_Position_Error;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//INS_Altitude_Error
    	ushTemp = IPON_MsgToSend.INS_Altitude_Error;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Roll_Error_RMS
    	ushTemp = IPON_MsgToSend.Roll_Error_RMS / ((double)((((2047.0) - (0.0)) / ((20470.0) - (0.0)))));
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Pitch_Error_RMS
    	ushTemp = IPON_MsgToSend.Pitch_Error_RMS / ((double)((((2047.0) - (0.0)) / ((20470.0) - (0.0)))));
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Number_Of_Satellites
    	uchTemp = IPON_MsgToSend.Number_Of_Satellites;
    	memcpy(pointToBuffer, (char*)&ushTemp, 1);
    	pointToBuffer+=1;

    	//Figure_Of_Merit
    	uchTemp = IPON_MsgToSend.Figure_Of_Merit;
    	memcpy(pointToBuffer, (char*)&ushTemp, 1);
    	pointToBuffer+=1;

    	//week
    	ushTemp = IPON_MsgToSend.week;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//GPS_UTC_offset
    	ushTemp = IPON_MsgToSend.GPS_UTC_offset;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Bytes_2_reserved
    	ushTemp = IPON_MsgToSend.Bytes_2_reserved;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;

    	//Checksum_Egi
    	ushTemp = IPON_MsgToSend.Checksum_Egi;
    	ushTemp = littleEndianToBig<unsigned short>(ushTemp);
    	memcpy(pointToBuffer, (char*)&ushTemp, 2);
    	pointToBuffer+=2;
	}

    void sendIMUToUDP_100HZMESSAGE(const common::UpdateInfo & _info)
    {

		math::Pose pose=_gps_ant_link->GetWorldPose();
		gazebo::math::Vector3 pos = pose.pos;

    	double roll, pitch, yaw = 0;
	    tf::Quaternion q(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
		tf::Matrix3x3 rot_mat(q);
		rot_mat.getEulerYPR(yaw,pitch,roll);

    	PHSPERIODIC100HZMESSAGE IPON_MsgToSend;

    	bzero(&IPON_MsgToSend, sizeof(PHSPERIODIC100HZMESSAGE));

    	IPON_MsgToSend.Message_ID_Accepted_From_EGI		= E_MESSAGE_ID_ACCEPTED_FROM_EGI::E_MESSAGE_ID_ACCEPTED_FROM_EGI_PERIODIC_100HZ; // ID for 100Hz message

        yaw = -yaw;
        if(yaw < 0)
          yaw += (2 * M_PI);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Azimuth_PD_geographic, yaw*Rad2Mills, 0, 6399.9);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Pitch_PD_Egi, (-pitch) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Roll_PD_Egi, roll * Rad2Mills, -1600, 1600);
#if GAZEBO_MAJOR_VERSION >= 6
	//Will be deprecated Gazebo 8
	/*
        insertBetweenRanges<double, double>(IPON_MsgToSend.Azimuth_rate_Z_PD_Egi, ((math::Vector3(_imu->AngularVelocity())).z+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Pitch_rate_Y_PD_Egi, ((math::Vector3(_imu->AngularVelocity())).y+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Roll_rate_X_PD_Egi, ((math::Vector3(_imu->AngularVelocity())).x+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
	*/
	insertBetweenRanges<double, double>(IPON_MsgToSend.Azimuth_rate_Z_PD_Egi, ((-_imu->AngularVelocity()).Z()+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Pitch_rate_Y_PD_Egi, ((-_imu->AngularVelocity()).Y()+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Roll_rate_X_PD_Egi, ((_imu->AngularVelocity()).X()+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
    	//ROS_INFO("Azimuth_rate_Z_PD_Egi: %f, Pitch_rate_Y_PD_Egi: %f, Roll_rate_X_PD_Egi: %f", IPON_MsgToSend.Azimuth_rate_Z_PD_Egi, IPON_MsgToSend.Pitch_rate_Y_PD_Egi, IPON_MsgToSend.Roll_rate_X_PD_Egi);

        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_X_Egi, (math::Vector3(_imu->LinearAcceleration())).x+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_Y_Egi, (math::Vector3(_imu->LinearAcceleration())).y+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_Z_Egi, (math::Vector3(_imu->LinearAcceleration())).z+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
#else
       
        insertBetweenRanges<double, double>(IPON_MsgToSend.Azimuth_rate_Z_PD_Egi, (-_imu->GetAngularVelocity().z+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Pitch_rate_Y_PD_Egi, (-_imu->GetAngularVelocity().y+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Roll_rate_X_PD_Egi, (_imu->GetAngularVelocity().x+_gy_bias+_gy_noise*sampleNormal()) * Rad2Mills, -1600, 1600);
    	//ROS_INFO("Azimuth_rate_Z_PD_Egi: %f, Pitch_rate_Y_PD_Egi: %f, Roll_rate_X_PD_Egi: %f", IPON_MsgToSend.Azimuth_rate_Z_PD_Egi, IPON_MsgToSend.Pitch_rate_Y_PD_Egi, IPON_MsgToSend.Roll_rate_X_PD_Egi);

        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_X_Egi, _imu->GetLinearAcceleration().x+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_Y_Egi, _imu->GetLinearAcceleration().y+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
        insertBetweenRanges<double, double>(IPON_MsgToSend.ECC_Z_Egi, _imu->GetLinearAcceleration().z+_acc_bias+_acc_noise*sampleNormal(), -50, 50);
#endif
        //ROS_INFO("ECC_X_Egi: %f, ECC_Y_Egi: %f, ECC_Z_Egi: %f", IPON_MsgToSend.ECC_X_Egi, IPON_MsgToSend.ECC_Y_Egi, IPON_MsgToSend.ECC_Z_Egi);

    	IPON_MsgToSend.Alt_correction_Egi				= 18; // defiantly not used in SAHAR				//value from the real IPON
    	IPON_MsgToSend.Checksum_Egi						= 0; // defiantly not used in SAHAR
    	//ROS_INFO("Alt_correction_Egi: %f, Altitude_MSL_EGI: %f", IPON_MsgToSend.Alt_correction_Egi, IPON_MsgToSend.Altitude_MSL_EGI);

    	bzero(IPON_MsgToSend.Eight_byte_Spare, 8); 		// defiantly not used in SAHAR

    	//INS_Time_Of_Nav_Data is the second count begins with "0" each Sunday morning at midnight, Zulu time (Greenwich time)
    	//At start up until GPS is available, INS time stars from 0
    	IPON_MsgToSend.INS_Time_Of_Nav_Data				= 1;

    	IPON_MsgToSend.Input_Message_Number_Echo_Egi	= 0; // defiantly not used in SAHAR

        LatLonAlt currentLatLonAlt = getLanLonAlt();
        insertBetweenRanges<double, float>(IPON_MsgToSend.Altitude_MSL_EGI, currentLatLonAlt.altitude, -16384, 16383);
        IPON_MsgToSend.LAT_Egi							= currentLatLonAlt.latitude * Deg2Pi; // Range: [-1.0, 1.0]
    	IPON_MsgToSend.LONG_Egi							= currentLatLonAlt.longitude * Deg2Pi; // Range: [-1.0, 1.0]
    	//ROS_INFO("LAT_Egi: %f, LONG_Egi:%f", IPON_MsgToSend.LAT_Egi, IPON_MsgToSend.LONG_Egi);

    	IPON_MsgToSend.SOM								= 42405; // defiantly not used in SAHAR (value is the default by the ICD)

    	IPON_MsgToSend.Validity_Word.Antena_Fail_Egi	= 0;
    	IPON_MsgToSend.Validity_Word.External_Voltage_Fail_Egi = 0;
    	IPON_MsgToSend.Validity_Word.GPS_Data_Not_Valid_Egi = 1;
    	IPON_MsgToSend.Validity_Word.GPS_Fail_Egi		= 0;
    	IPON_MsgToSend.Validity_Word.IMU_Fail_Egi		= 0;
    	IPON_MsgToSend.Validity_Word.INS_Fail_Egi		= 0;
    	IPON_MsgToSend.Validity_Word.NA1_Egi			= 0;
    	IPON_MsgToSend.Validity_Word.NA2_Egi			= 0;
    	IPON_MsgToSend.Validity_Word.NA3_Egi			= 0;
    	IPON_MsgToSend.Validity_Word.NA4_Egi			= 0;
    	IPON_MsgToSend.Validity_Word.NA5_Egi			= 0;
    	IPON_MsgToSend.Validity_Word.NA_Egi				= 0;
    	IPON_MsgToSend.Validity_Word.ODO_Fail_Egi		= 0;
    	IPON_MsgToSend.Validity_Word.ONE_PPS_Fail_Egi	= 0;
    	IPON_MsgToSend.Validity_Word.Severe_INS_Fail_Egi= 0;
    	IPON_MsgToSend.Validity_Word.Temperature_Fail_Egi = 1;

        insertBetweenRanges<double, double>(IPON_MsgToSend.Velocity_East_Egi, (-getLanLonAlt_Velocities().y), -128, 127);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Velocity_north_Egi, getLanLonAlt_Velocities().x, -128, 127);
        insertBetweenRanges<double, double>(IPON_MsgToSend.Velocity_down_Egi, (-getLanLonAlt_Velocities().z), -128, 127);

    	IPON_MsgToSend.blockLength						= 60; // defiantly not used in SAHAR (value is the default by the ICD)

        bzero(bufferOf100HZMsg, 1000);
        unsigned char* pointToBuffer;
        char* pointToSrc;
        double dTemp = 0;
        unsigned int uiTemp = 0;
        int iTemp = 0;
        short shTemp = 0;
        unsigned short ushTemp = 0;
        pointToBuffer = bufferOf100HZMsg;

        //SUM
        ushTemp = littleEndianToBig<unsigned short>(IPON_MsgToSend.SOM);
        memcpy(pointToBuffer, &ushTemp, 2);
        pointToBuffer+=2;

        //Message_ID_Accepted_From_EGI
        iTemp = littleEndianToBig<E_MESSAGE_ID_ACCEPTED_FROM_EGI>(IPON_MsgToSend.Message_ID_Accepted_From_EGI);
        pointToSrc = (char*)&iTemp;
        pointToSrc+=3;
        memcpy(pointToBuffer, pointToSrc, 1);
        pointToBuffer+=1;
        pointToSrc = NULL;

        //blockLength
        ushTemp = IPON_MsgToSend.blockLength;
        ushTemp = littleEndianToBig<unsigned short>(ushTemp);
        pointToSrc = (char*)&ushTemp;
        pointToSrc+=1;
        memcpy(pointToBuffer, pointToSrc, 1);
        pointToBuffer+=1;
        pointToSrc = NULL;

        //INS_Time_Of_Nav_Data
        uiTemp = IPON_MsgToSend.INS_Time_Of_Nav_Data / ((double)((((1048576.0) - (0.0)) / ((4294967295.0) - (0.0)))));
        uiTemp = littleEndianToBig<unsigned int>(uiTemp);
        memcpy(pointToBuffer, (char*)&uiTemp, 4);
        pointToBuffer+=4;

        //LAT_Egi
        dTemp = (IPON_MsgToSend.LAT_Egi - ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))))) / ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483647.0)))));
        iTemp = dTemp;
        iTemp = littleEndianToBig<int>(iTemp);
        memcpy(pointToBuffer, (char*)&iTemp, 4);
        pointToBuffer+=4;

        //LONG_Egi
        dTemp = (IPON_MsgToSend.LONG_Egi - ((double)((-1.0 - (-2147483648.0 * (((1.0) - (-1.0)) / ((2147483647.0) - (-2147483648.0)))))))) / ((double)((((1.0) - (-1.0)) / ((2147483647.0) - (-2147483647.0)))));
        iTemp = dTemp;
        iTemp = littleEndianToBig<int>(iTemp);
        memcpy(pointToBuffer, (char*)&iTemp, 4);
        pointToBuffer+=4;

        //Altitude_MSL_EGI
        dTemp = IPON_MsgToSend.Altitude_MSL_EGI / ((double) ((((16383.0) - (-16384.0)) / ((163830.0) - (-163840.0)))));
        iTemp = dTemp;
        iTemp = littleEndianToBig<int>(iTemp);
        memcpy(pointToBuffer, (char*)&iTemp, 4);
        pointToBuffer+=4;

        //Pitch_PD_Egi
        dTemp = IPON_MsgToSend.Pitch_PD_Egi / ((double) ((((1600.0) - (-1600.0)) / ((16000.0) - (-16000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Roll_PD_Egi
        dTemp = IPON_MsgToSend.Roll_PD_Egi / ((double) ((((1600.0) - (-1600.0)) / ((16000.0) - (-16000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Azimuth_PD_geographic
        dTemp = IPON_MsgToSend.Azimuth_PD_geographic / ((double) ((((6400.0) - (0.0)) / ((64000.0) - (0.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Roll_rate_X_PD_Egi
        dTemp = IPON_MsgToSend.Roll_rate_X_PD_Egi / ((double) ((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Pitch_rate_Y_PD_Egi
        dTemp = IPON_MsgToSend.Pitch_rate_Y_PD_Egi / ((double) ((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Azimuth_rate_Z_PD_Egi
        dTemp = IPON_MsgToSend.Azimuth_rate_Z_PD_Egi / ((double) ((((1600.0) - (-1600.0)) / ((32000.0) - (-32000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //ECC_X_Egi
        dTemp = IPON_MsgToSend.ECC_X_Egi / ((double) ((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //ECC_Y_Egi
        dTemp = IPON_MsgToSend.ECC_Y_Egi / ((double) ((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //ECC_Z_Egi
        dTemp = IPON_MsgToSend.ECC_Z_Egi / ((double) ((((50.0) - (-50.0)) / ((5000.0) - (-5000.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Velocity_north_Egi
        dTemp = IPON_MsgToSend.Velocity_north_Egi / ((double) ((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Velocity_East_Egi
        dTemp = IPON_MsgToSend.Velocity_East_Egi / ((double) ((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Velocity_down_Egi
        dTemp = IPON_MsgToSend.Velocity_down_Egi / ((double) ((((127.0) - (-128.0)) / ((32512.0) - (-32768.0)))));
        shTemp = dTemp;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Alt_correction_Egi
        shTemp = IPON_MsgToSend.Alt_correction_Egi;
        shTemp = littleEndianToBig<short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;

        //Validity_Word
        bzero(&shTemp, 2);
        shTemp |= IPON_MsgToSend.Validity_Word.GPS_Data_Not_Valid_Egi << 15;
        shTemp |= IPON_MsgToSend.Validity_Word.Temperature_Fail_Egi << 14;
        shTemp |= IPON_MsgToSend.Validity_Word.NA_Egi << 13;
        shTemp |= IPON_MsgToSend.Validity_Word.NA1_Egi << 12;
        shTemp |= IPON_MsgToSend.Validity_Word.NA2_Egi << 11;
        shTemp |= IPON_MsgToSend.Validity_Word.NA3_Egi << 10;
        shTemp |= IPON_MsgToSend.Validity_Word.NA4_Egi << 9;
        shTemp |= IPON_MsgToSend.Validity_Word.NA5_Egi << 8;
        shTemp |= IPON_MsgToSend.Validity_Word.Severe_INS_Fail_Egi << 7;
        shTemp |= IPON_MsgToSend.Validity_Word.External_Voltage_Fail_Egi << 6;
        shTemp |= IPON_MsgToSend.Validity_Word.Antena_Fail_Egi << 5;
        shTemp |= IPON_MsgToSend.Validity_Word.ONE_PPS_Fail_Egi << 4;
        shTemp |= IPON_MsgToSend.Validity_Word.IMU_Fail_Egi << 3;
        shTemp |= IPON_MsgToSend.Validity_Word.GPS_Fail_Egi << 2;
        shTemp |= IPON_MsgToSend.Validity_Word.ODO_Fail_Egi << 1;
        shTemp |= IPON_MsgToSend.Validity_Word.INS_Fail_Egi << 0;

        shTemp = littleEndianToBig<unsigned short>(shTemp);
        memcpy(pointToBuffer, (char*)&shTemp, 2);
        pointToBuffer+=2;
        pointToSrc = NULL;

        //Input_Message_Number_Echo_Egi
        dTemp = IPON_MsgToSend.Input_Message_Number_Echo_Egi;
        ushTemp = dTemp;
        ushTemp = littleEndianToBig<unsigned short>(ushTemp);
        memcpy(pointToBuffer, (char*)&ushTemp, 2);
        pointToBuffer+=2;
        pointToSrc = NULL;

        //Eight_byte_Spare
        bzero(pointToBuffer, 8);
        pointToBuffer+=8;

        //Checksum_Egi
        ushTemp = IPON_MsgToSend.Checksum_Egi;
        ushTemp = littleEndianToBig<unsigned short>(ushTemp);
        memcpy(pointToBuffer, (char*)&ushTemp, 2);
        pointToBuffer+=2;
        pointToSrc = NULL;

        std::pair<common::Time, char[1000]> *pairToPush = new (std::pair<common::Time, char[1000]>);
        pairToPush->first = sim_Time;
        memcpy(pairToPush->second, bufferOf100HZMsg, 1000);
   		vecToSend_mutex.lock();
        vecToSend->insert(vecToSend->end(),pairToPush);
   		vecToSend_mutex.unlock();


    }
    template <typename T>
    T littleEndianToBig(T u)
    {
    	union
    	{
    		T u;
    		unsigned char u8[sizeof(T)];
    	}source,dest;
    	source.u = u;
    	for (size_t k=0; k<sizeof(T); k++)
    		dest.u8[k] = source.u8[sizeof(T)-k-1];
    	return dest.u;
    }

  private:
    physics::ModelPtr 					_model; // Pointer to the model
    physics::LinkPtr 					_gps_ant_link;

    boost::mutex 						vecToSend_mutex;

    event::ConnectionPtr 				_updateConnection; // Pointer to the update event connection

    sensors::ImuSensorPtr 				_imu;

    std::string 						_gps_ant_link_name;
    std::string 						_imu_unit_name;

    math::Vector3 						_init_pos;
    double 								_start_latitude, _start_longitude;
    double 								_gps_noise,_rp_noise, _yaw_noise, _gy_noise, _acc_noise, _acc_bias, _gy_bias, _spd_noise;
    int  								_frequency;
    common::Time						sim_Time; //used for timing debug
    common::Time						lastTimeOf100HZMSGSending;
    common::Time						lastTimeOf1HZMSGSending;
    int 								_seq;

    //socket
	int 								UDPsockfd, UDPPortno;
	int 								TCPsockfd, TCPPortno;

	//socklen_t clilen;
    unsigned char 						bufferOf100HZMsg[1000];
    unsigned char 						bufferOf1HZMsg[1000];
    struct sockaddr_in 					UDPserv_addr;
    struct sockaddr_in 					TCPserv_addr;

    std::atomic<bool> 					UDPSocketIsConnected;

    boost::thread 						_udpSenderThread;
    std::atomic<bool>					shouldSend1HZMsg;

    std::vector<std::pair<common::Time, char[1000]> *> *vecToSend;

    string IP;
    double Delay;
    double FC_sumOfFrequencyForAverage;
    int FC_counterOfSecondsForFrequencyAverage;
    int FC_counterOfMsgInSec;
    int FC_LastTime_Second;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(IPON)
}
