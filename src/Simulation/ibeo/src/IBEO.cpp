

/*
 * <INSERT FAKE LICENSE HERE>
 */

/*
 * Desc: LD-MRS Plugin for the Sahar Project
 * Author: Daniel Meltz
 */
#include "../include/IBEO.h"
#ifndef USE_GPU
#define USE_GPU 1

//using namespace gazebo;
using namespace std;

namespace gazebo
{   
  class IBEO : public ModelPlugin
  {
    public:
	  ~IBEO()
	  {
			shutdown(newsockfd,SHUT_RDWR);
			shutdown(sockfd,SHUT_RDWR);
	  }


    //loadParametersFromSDFFile
	void loadParametersFromSDFFile(sdf::ElementPtr _sdf)
	    {
	        if( ! (_sdf->HasElement("Robot_Name_Space")) && (_sdf->HasElement("Sensor_Name"))
	      		&& (_sdf->HasElement("row_t1_name")) && (_sdf->HasElement("row_t2_name")) && (_sdf->HasElement("row_b1_name")) && (_sdf->HasElement("row_b2_name"))
	      		&& (_sdf->HasElement("row_t1_pitch_ang")) && (_sdf->HasElement("row_t2_pitch_ang")) && (_sdf->HasElement("row_b1_pitch_ang")) && (_sdf->HasElement("row_b2_pitch_ang"))
	      		&& (_sdf->HasElement("rows_t_yaw_ang_min")) && (_sdf->HasElement("rows_t_yaw_ang_max")) && (_sdf->HasElement("rows_b_yaw_ang_min")) && (_sdf->HasElement("rows_b_yaw_ang_max"))
	      	    && (_sdf->HasElement("rows_yaw_ang_increment"))
	      	    && (_sdf->HasElement("scanning_frequency"))
	      	    && (_sdf->HasElement("distance_sample_resolution")) && (_sdf->HasElement("distance_min")) && (_sdf->HasElement("distance_max"))
	      	    && (_sdf->HasElement("_TF_parent_link_name"))
	      	    && (_sdf->HasElement("laser_TF_point_of_origin_X")) && (_sdf->HasElement("laser_TF_point_of_origin_Y")) && (_sdf->HasElement("laser_TF_point_of_origin_Z"))
	      	    && (_sdf->HasElement("laser_TF_point_of_origin_Rol")) && (_sdf->HasElement("laser_TF_point_of_origin_Pit")) && (_sdf->HasElement("laser_TF_point_of_origin_Yow")) )
	  		{
			  string error = "IBEO Sensor Plugin ERROR - missing input parameters -\n";
			  gzthrow(error);
			  return;
	  		}

	        Delay = 0.02;
	        if (_sdf->HasElement("Delay"))
	        	_sdf->GetElement("Delay")->GetValue()->Get(Delay);

	        _sdf->GetElement("Robot_Name_Space")->GetValue()->Get<std::string>(_Robot_Name_Space);
	        _sdf->GetElement("Sensor_Name")->GetValue()->Get<std::string>(_Sensor_Name);

	        _sdf->GetElement("row_t1_name")->GetValue()->Get<std::string>(_sensor_t1_name);
	        _sdf->GetElement("row_t2_name")->GetValue()->Get<std::string>(_sensor_t2_name);
	        _sdf->GetElement("row_b1_name")->GetValue()->Get<std::string>(_sensor_b1_name);
	        _sdf->GetElement("row_b2_name")->GetValue()->Get<std::string>(_sensor_b2_name);

	        _row_t1_pitch_ang = _sdf->GetElement("row_t1_pitch_ang")->Get<float>();
	        _row_t2_pitch_ang = _sdf->GetElement("row_t2_pitch_ang")->Get<float>();
	        _row_b1_pitch_ang = _sdf->GetElement("row_b1_pitch_ang")->Get<float>();
	        _row_b2_pitch_ang = _sdf->GetElement("row_b2_pitch_ang")->Get<float>();

	        _rows_t_end_ang = _sdf->GetElement("rows_t_end_ang")->Get<float>();
	        _rows_t_start_ang = _sdf->GetElement("rows_t_start_ang")->Get<float>();
	        _rows_b_end_ang = _sdf->GetElement("rows_b_end_ang")->Get<float>();
	        _rows_b_start_ang = _sdf->GetElement("rows_b_start_ang")->Get<float>();

	  	   _rows_yaw_ang_increment = _sdf->GetElement("rows_yaw_ang_increment")->Get<float>();
	  	   _scanning_frequency = _sdf->GetElement("scanning_frequency")->Get<double>();
	  	   _distance_sample_resolution = _sdf->GetElement("distance_sample_resolution")->Get<float>();
	  	   _distance_min = _sdf->GetElement("distance_min")->Get<float>();
	  	   _distance_max = _sdf->GetElement("distance_max")->Get<float>();

	  	   _sdf->GetElement("TF_parent_link_name")->GetValue()->Get<std::string>(_TF_parent_link_name);
	  	  float origin_X = _sdf->GetElement("laser_TF_point_of_origin_X")->Get<float>();
	  	  float origin_Y = _sdf->GetElement("laser_TF_point_of_origin_Y")->Get<float>();
	  	  float origin_Z = _sdf->GetElement("laser_TF_point_of_origin_Z")->Get<float>();
	  	  float origin_Rol = _sdf->GetElement("laser_TF_point_of_origin_Rol")->Get<float>();
	  	  float origin_Pit = _sdf->GetElement("laser_TF_point_of_origin_Pit")->Get<float>();
	  	  float origin_Yow = _sdf->GetElement("laser_TF_point_of_origin_Yow")->Get<float>();

		  _laser_TF_point_of_origin.setOrigin( tf::Vector3(origin_X,origin_Y,origin_Z));
		  _laser_TF_point_of_origin.setRotation( tf::Quaternion(0,0,0,1));

	    }

	//initSensors
	void initSensors()
	{
		  sensors::SensorPtr sensorB1, sensorB2, sensorT1, sensorT2;

		  sensorT1 = sensors::SensorManager::Instance()->GetSensor(_sensor_t1_name);
		  sensorT2 = sensors::SensorManager::Instance()->GetSensor(_sensor_t2_name);
		  sensorB1 = sensors::SensorManager::Instance()->GetSensor(_sensor_b1_name);
		  sensorB2 = sensors::SensorManager::Instance()->GetSensor(_sensor_b2_name);

		  if(!sensorB1 || !sensorB2 || !sensorT1 || !sensorT2)
		  {
			string error = "IBEO Sensor Model \"" + _model->GetName() + "\" failed to locate some of his 4 sub-sensors.\n(Do the names in the .sdf match the names in the .cpp?)\n";
			gzthrow(error);
			return;
		  }
#if GAZEBO_MAJOR_VERSION >= 7
#ifdef USE_GPU
		{
		  _sensorB1 = std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB1);
		  _sensorB2 = std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB2);
		  _sensorT1 = std::static_pointer_cast<sensors::GpuRaySensor>(sensorT1);
		  _sensorT2 = std::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorT2);
		}
#else
		{
		  _sensorB1 = std::dynamic_pointer_cast<sensors::RaySensor>(sensorB1);
		  _sensorB2 = std::dynamic_pointer_cast<sensors::RaySensor>(sensorB2);
		  _sensorT1 = std::static_pointer_cast<sensors::RaySensor>(sensorT1);
		  _sensorT2 = std::dynamic_pointer_cast<sensors::RaySensor>(sensorT2);
		}
#endif
#else
#ifdef USE_GPU
	      {
		 _sensorB1 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB1);
		 _sensorB2 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorB2);
		 _sensorT1 = boost::static_pointer_cast<sensors::GpuRaySensor>(sensorT1);
		 _sensorT2 = boost::dynamic_pointer_cast<sensors::GpuRaySensor>(sensorT2);
	      }
#else
	      {
		  _sensorB1 = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorB1);
		  _sensorB2 = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorB2);
		  _sensorT1 = boost::static_pointer_cast<sensors::RaySensor>(sensorT1);
		  _sensorT2 = boost::dynamic_pointer_cast<sensors::RaySensor>(sensorT2);
	      }
#endif
#endif
	      if(!_sensorB1 || !_sensorB2 || !_sensorT1 || !_sensorT2)
	      {
			string error = "IBEO Sensor Model \"" + _model->GetName() + "\" found that those sensors are not of class GpuRaySensor. Strange\n";
			gzthrow(error);
			return;
	      }
	}

	//initSocket
	void initSocket()
	{
	  
	  sockfd = socket(AF_INET,SOCK_STREAM | SOCK_NONBLOCK,0);
	  ROS_INFO("sockfd = %d", sockfd);
	  if(sockfd<0)
	  {
	    // ROS_ERROR("IBEO socket not connected");
	    gzerr << "Cannot create socket errno=" << errno << "\n";
	    exit(1);
	  }

	  memset((char *) & serv_addr, 0,sizeof(serv_addr));
	  portno= atoi("12002");
	  serv_addr.sin_family=AF_INET;
	  serv_addr.sin_addr.s_addr=htonl(INADDR_ANY);
	  serv_addr.sin_port=htons(portno);

	  int flag =1;
	  setsockopt(sockfd, SOL_SOCKET,SO_REUSEADDR ,&serv_addr,sizeof(serv_addr));
	  setsockopt(sockfd, IPPROTO_TCP,TCP_NODELAY ,&flag,sizeof(int));
//	  setsockopt(sockfd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(int));

	  if(bind(sockfd,(struct sockaddr*) &serv_addr, sizeof(serv_addr))<0)
	  {
	    gzerr << "IBEO socket bind fail errno=" << errno << "\n" ;
	    exit(1);
	  }
	  ROS_INFO("sockfd = %d", sockfd);
	}

	//Load
        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
	{
	  this->_model = _parent;
	  rviz_points_index=0;
	  client_connected=false;
//		  initMsgWasSend=false;
	  sockfd = -1;
	  newsockfd = -1;
	  flag_fillMsg = false;

          loadParametersFromSDFFile(_sdf);

          initSensors();
#if GAZEBO_MAJOR_VERSION >= 7
          numOfIbeoPoints = _sensorB1->RangeCount()+_sensorB2->RangeCount()+_sensorT1->RangeCount()+_sensorT2->RangeCount();
#else
          numOfIbeoPoints = _sensorB1->GetRangeCount()+_sensorB2->GetRangeCount()+_sensorT1->GetRangeCount()+_sensorT2->GetRangeCount();
#endif
          this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&IBEO::OnUpdate, this, _1));
          _marker_pub = _nodeHandle.advertise<visualization_msgs::Marker>(_Robot_Name_Space+"/"+_Sensor_Name+"/markers", 10);
          
          initSocket();

	  _tcpSenderThread=boost::thread(&IBEO::sendThreadMethod,this);
	  _controlThread=boost::thread(&IBEO::controlThread,this);

          vecToSend = new (std::vector<std::pair<common::Time, char[10000]> *>);

          FC_LastTime_Second = 0;
          FC_sumOfFrequencyForAverage = 0;
          FC_counterOfSecondsForFrequencyAverage = 0;
          FC_counterOfMsgInSec = 0;
        }

	void controlThread()
	{
		int d = 0;
		while(true)
		{
			sleep(1);
			if(vecToSend)
				d = vecToSend->size();
			if(d>10)
			{
				client_connected=false;
				shutdown(newsockfd,SHUT_RDWR);
				newsockfd=-1;
				ROS_INFO("control thread closed the socket");
			}
		}
	}

	// This thread is used for situation where the parser on the side of the Robot, before transit to receiving data state performs sequence of seting parameters of the sensor
	// we ignore the parameters that the parser requires and only replay with (false) acknowledge messages
	void responseThreadMethhod()
	{
	  char buffer[500];
	  unsigned int sizeOfData;
	  while(client_connected && newsockfd >= 0)
	  {

	    int n = 0;
	    int rv = poll(&ufds[0], 1, -1);
	    if(rv == -1)
	      ROS_INFO("IBEO: error in poll() at responseThreadMethhod()");
	    if(ufds[0].revents & POLLIN)
	    {
	      bzero(buffer, 500);
	      n = recv(newsockfd, buffer, sizeof(ibeoScanDataHeader), 0);
	      sizeOfData = 0;
	      memcpy(&sizeOfData, buffer+8, 4);
	      sizeOfData = littleEndianToBig<unsigned int>(sizeOfData);
	      n += recv(newsockfd, &buffer[sizeof(ibeoScanDataHeader)], sizeOfData, 0);
	    }
	    else
	    {
	      continue;
	    }

	    if(strlen(buffer) == 0 || n < 0)
	    {
	      continue;
	    }

	    int magicWord = 0;
	    memcpy(&magicWord, buffer, 4);
	    if (magicWord != littleEndianToBig<int>(0xaffec0c2))
	    {
	      continue;
	    }


            ReplayMSG replay;
            replay.Header.MagicWord = littleEndianToBig<int>(0xaffec0c2);
            replay.Header.SizePreviousMessage = sizeOfData;//0; // littleEndianToBig was done after filling the value
            replay.Header.SizeCurrentMessage=littleEndianToBig<unsigned int>(sizeof(ReplayMSG) - sizeof(ibeoScanDataHeader) );
            replay.Header.Reserved = 0;
            replay.Header.DeviceID = 0;
            replay.Header.DataType = littleEndianToBig<unsigned short>(0x2020);
            replay.Header.time_up = 0;    // NTP64
            replay.Header.time_down = 0;  // NTP64
            memcpy(buffer+24, &replay.commandID, 2); //copy the command id from the received msg

	    int second_rv = poll(&ufds[1], 1, -1);
	    if(second_rv == -1)
		ROS_INFO("IBEO: error in poll() at sendThreadMethod()");
	    if(ufds[1].revents & POLLOUT)
	    {
	      n = sendto(newsockfd,&replay,sizeof(ReplayMSG),0,(sockaddr *)&cli_addr,clilen);
	    }
	  }
	}

	void checkSocket(int sockfdToCheck)
	{
		int error = 0;
		socklen_t len = sizeof(error);
		int retval = getsockopt(sockfdToCheck, SOL_SOCKET, SO_ERROR, &error, &len);
		if(retval != 0)
		{
			//there was a problem getting the error code
			printf("error getting socket error code: %s\n", strerror(retval));
		}
		if(error!=0)
		{
			//socket has a non zero error status
			printf("socket error: %s\n", strerror(error));
		}
	}

	//sendThreadMethod
	void sendThreadMethod()
	{
	  while(true)
	  {
    		if(vecToSend && !vecToSend->empty())
    		{
    	   		vecToSend_mutex.lock();
    	   		std::pair<common::Time, char[10000]>* front = (vecToSend->front());
    	   		vecToSend_mutex.unlock();
    	   		if(!front)
    	   			continue;

    	   		double diff = (front->first.Double() + Delay) - sim_Time.Double() ;
				if(diff >= 0.00001)
					continue;

    		    int bytes_sent=0;
    			while(bytes_sent<(sizeof(SibeoScanData) +(numOfIbeoPoints*sizeof(IbeoScanPoint)) )&& client_connected /*&& initMsgWasSend*/)
			    {
    				int rv = poll(&ufds[1], 1, -1);
    				if(rv == -1)
    					ROS_INFO("IBEO: error in poll() at sendThreadMethod()");
    				if(ufds[1].revents & POLLOUT)
    				{
					   int n = sendto(newsockfd,front->second+bytes_sent,(sizeof(SibeoScanData) + (numOfIbeoPoints*sizeof(IbeoScanPoint))) - bytes_sent,0/*x08*/,(sockaddr *)&cli_addr,clilen);
					   if(n>0)
					   {
						   bytes_sent+=n;
					   }
					   if(n<0)
					   {
						   //ROS_INFO("NOT SEND, errno = %d, %s", errno, strerror(errno));
						   if(newsockfd>0)
						   {
							   shutdown(newsockfd,SHUT_RDWR);
							   newsockfd=-1;
						   }
						   client_connected=false;
						   break;
					   }
    				}
			    }
    			vecToSend_mutex.lock();
				vecToSend->erase( (vecToSend->begin()) );
		    	vecToSend_mutex.unlock();
				delete front;

				// frequency checking - if average of frequency is not correct for 10 seconds warn message will appear
      	  		if (sim_Time.sec - FC_LastTime_Second >= 1 )
      	  		{
      	  		   FC_counterOfMsgInSec++;
  	  			   FC_LastTime_Second = sim_Time.sec;
  	  			   FC_sumOfFrequencyForAverage += FC_counterOfMsgInSec;
  	  			   FC_counterOfSecondsForFrequencyAverage++;

  	  			   //ROS_INFO("IBEO msg/sec: %d \n\n", FC_counterOfMsgInSec);
  	  			   FC_counterOfMsgInSec = 0;

  	  			   if(FC_counterOfSecondsForFrequencyAverage == 10)
  	  			   {
  	  				  if((FC_sumOfFrequencyForAverage / FC_counterOfSecondsForFrequencyAverage) - ((double)_scanning_frequency) != 0)
  	  					  ROS_WARN("IBEO: error in frequency, Frequency Average = %f and it's should be = %f", FC_sumOfFrequencyForAverage / FC_counterOfSecondsForFrequencyAverage, (double) _scanning_frequency);

        	  			  FC_sumOfFrequencyForAverage = 0;
        	  			  FC_counterOfSecondsForFrequencyAverage = 0;
        	  		   }
        	  	}
      	  		else
      	  		{
      	  			FC_counterOfMsgInSec++;
      	  		}
				// end frequency checking
    		}
	  }
	}

	//getRanges
	void getRanges(vector<double>& rangesT1, vector<double>& rangesT2, vector<double>& rangesB1, vector<double>& rangesB2)
	{
#if GAZEBO_MAJOR_VERSION >= 7
        _sensorB1->Ranges(rangesB1);
        _sensorB2->Ranges(rangesB2);
        _sensorT1->Ranges(rangesT1);
        _sensorT2->Ranges(rangesT2);
#else
	_sensorB1->GetRanges(rangesB1);
        _sensorB2->GetRanges(rangesB2);
        _sensorT1->GetRanges(rangesT1);
        _sensorT2->GetRanges(rangesT2);
#endif
        float Dm = _distance_min;
        float Dr = _distance_sample_resolution;

        for(unsigned int i = 0; i < rangesT1.size(); ++i)
        	rangesT1[i] = Dm + ((int)((rangesT1[i]-Dm + Dr/2) / Dr)) * Dr;
        for(unsigned int i = 0; i < rangesT2.size(); ++i)
        	rangesT2[i] = Dm + ((int)((rangesT2[i]-Dm + Dr/2) / Dr)) * Dr;
        for(unsigned int i = 0; i < rangesB1.size(); ++i)
        	rangesB1[i] = Dm + ((int)((rangesB1[i]-Dm + Dr/2) / Dr)) * Dr;
        for(unsigned int i = 0; i < rangesB2.size(); ++i)
        	rangesB2[i] = Dm + ((int)((rangesB2[i]-Dm + Dr/2) / Dr)) * Dr;
	}

    //acceptClient
	bool acceptClient()
	{
      	  listen(sockfd,5);
      	  clilen = sizeof(sockaddr);
      	  newsockfd=accept(sockfd,(struct sockaddr *)&cli_addr,&clilen);
      	  if(newsockfd<0)
      	  {
      		  return false;
      	  }
      	  else
      	  {
      		  ufds[0].fd = newsockfd;
      		  ufds[0].events = POLLIN;//recv
      		  ufds[1].fd = newsockfd;
      		  ufds[1].events = POLLOUT;//send
                  client_connected = true;
      		  ROS_INFO("ibeo client connected");

                  _tcpRestartThread=boost::thread(&IBEO::responseThreadMethhod,this);
      	  }
      	  return true;
	}

    //fillPoints
	void fillPoints(int& pointCounter, unsigned char Layer_Echo, double yaw_ang_start, double yaw_ang_increment, vector<double> ranges)
	{
	  for(int i = 0;i < ranges.size();i++)
	  {
     	   m_ibeoScan.Point[pointCounter].Layer_Echo=		Layer_Echo;
     	   m_ibeoScan.Point[pointCounter].Flags=			66;
     	   m_ibeoScan.Point[pointCounter].HorizontalAngel= (short)( (yaw_ang_start + yaw_ang_increment * i)*(11520/(2*PI)) );
     	   m_ibeoScan.Point[pointCounter].RadialDistance=	(unsigned short)(ranges[ranges.size()-i]*100);
     	   m_ibeoScan.Point[pointCounter].EchoPulseWidth=	120;
     	   pointCounter++;
	  }
	}

    //fillMsg
	void fillMsg(vector<double> rangesT1, vector<double> rangesT2, vector<double> rangesB1, vector<double> rangesB2)
	{
	  m_ibeoScan.Header.MagicWord = littleEndianToBig<int>(0xaffec0c2);//0xc2c0feaf;
	  m_ibeoScan.Header.SizePreviousMessage = 0;
	  m_ibeoScan.Header.SizeCurrentMessage=littleEndianToBig<unsigned int>((sizeof(SibeoScanData) + (numOfIbeoPoints*sizeof(IbeoScanPoint))) - sizeof(ibeoScanDataHeader));

	  m_ibeoScan.Header.DeviceID = 0;
	  m_ibeoScan.Header.DataType = littleEndianToBig<unsigned short>(0x2202);
	  m_ibeoScan.Header.time_up = 0;	// NTP64
	  m_ibeoScan.Header.time_down = 0;  // NTP64
	  m_ibeoScan.Header.Reserved=0; //Reserved

	  m_ibeoScan.Scan.ScanNumber=3;
	  m_ibeoScan.Scan.ScannerStatus=0x20;
	  m_ibeoScan.Scan.SyncPhaseOffset=0;//(1.0/_sensorT1->GetUpdateRate()/700);

	  m_ibeoScan.Scan.ScanStratTimeDOWN=ros::Time::now().nsec;
	  m_ibeoScan.Scan.ScanStratTimeUP=ros::Time::now().sec;
	  m_ibeoScan.Scan.ScanEndTimeDOWN=ros::Time::now().nsec+1;
	  m_ibeoScan.Scan.ScanEndTimeUP=ros::Time::now().sec;

	  m_ibeoScan.Scan.AngelsTicks = 11520;//(0.0023);
	  m_ibeoScan.Scan.StartAngel = (short)(std::max(_rows_t_start_ang,_rows_b_start_ang) * (11520 / (2 * PI)));
	  m_ibeoScan.Scan.EndAngel = (short)(std::min(_rows_t_end_ang,_rows_b_end_ang) * (11520 / (2 * PI)));

	  m_ibeoScan.Scan.ScanPoints = rangesT1.size()+rangesT2.size()+rangesB1.size()+rangesB2.size();
	  m_ibeoScan.Scan.PositionYaw=0; //Reserved
	  m_ibeoScan.Scan.PositionPitch=0; //Reserved
	  m_ibeoScan.Scan.PositionRoll=0; //Reserved
	  m_ibeoScan.Scan.PositionX=0; //Reserved
	  m_ibeoScan.Scan.PositionY=0; //Reserved
	  m_ibeoScan.Scan.Reserved=0; //Reserved

	  int pointCounter=0;
	  int maxDist=0;

	  fillPoints(pointCounter, 0, _rows_b_start_ang , _rows_yaw_ang_increment, rangesB2);
	  fillPoints(pointCounter, 1, _rows_b_start_ang , _rows_yaw_ang_increment, rangesB1);
	  fillPoints(pointCounter, 2, _rows_t_start_ang , _rows_yaw_ang_increment, rangesT1);
	  fillPoints(pointCounter, 3, _rows_t_start_ang , _rows_yaw_ang_increment, rangesT2);
	}

    //OnUpdate
	void OnUpdate(const common::UpdateInfo & _info)
	{
	  sim_Time = _info.simTime;
	  double SensorTimeInterval = (double)((double)1.0/(double)_scanning_frequency );  // = 0.080sec
	  double dt = sim_Time.Double()-LastSensorUpdateTime.Double();


	  double diff_update_time = dt - SensorTimeInterval;

	  if (  diff_update_time >= -0.0001  ) //   using :  " >= 0 " will not work correctly in cases when TimeInterval is a whole number of simulation time steps !!!!!!
	  {
	    LastSensorUpdateTime = sim_Time;
	    flag_fillMsg = true;
	  }
#if GAZEBO_MAJOR_VERSION >= 7
	  double diff_update_B1 = _sensorB1->LastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_B2 = _sensorB2->LastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_T1 = _sensorT1->LastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_T2 = _sensorT2->LastMeasurementTime().Double() - LastSensorUpdateTime.Double();
#else
	  double diff_update_B1 = _sensorB1->GetLastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_B2 = _sensorB2->GetLastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_T1 = _sensorT1->GetLastMeasurementTime().Double() - LastSensorUpdateTime.Double();
	  double diff_update_T2 = _sensorT2->GetLastMeasurementTime().Double() - LastSensorUpdateTime.Double();
#endif
	  if ( (diff_update_B1 >= -0.0001 ) &&
    	   (diff_update_B2 >= -0.0001 ) &&
		   (diff_update_T1 >= -0.0001 ) &&
		   (diff_update_T2 >= -0.0001 ) &&
		   (std::abs(diff_update_B1 - diff_update_B2) <= 0.0001) &&
		   (std::abs(diff_update_B2 - diff_update_T1) <= 0.0001) &&
		   (std::abs(diff_update_T1 - diff_update_T2) <= 0.0001) &&
    	   (flag_fillMsg))
	  {

		  flag_fillMsg = false;

		  vector<double> rangesT1, rangesT2, rangesB1, rangesB2;
		  getRanges(rangesT1, rangesT2, rangesB1, rangesB2);
		  ros::Time new_scan_time_tag = ros::Time::now();
		  Publish_RVIZ_Message(rangesT1, rangesT2, rangesB1, rangesB2, new_scan_time_tag);
		  // BroadcastTF(new_scan_time_tag);

		  if(!client_connected && !acceptClient())
		  {
			  //return;
		  }

		  memset(&m_ibeoScan,0,sizeof(SibeoScanData));
		  m_ibeoScan.Point = new IbeoScanPoint[numOfIbeoPoints];
		  bzero(m_ibeoScan.Point, sizeof(IbeoScanPoint)*numOfIbeoPoints);

		  fillMsg(rangesT1, rangesT2, rangesB1, rangesB2);

		  std::pair<common::Time, char[10000]> *pairToPush = new (std::pair<common::Time, char[10000]>);
		  pairToPush->first = sim_Time;
		  memcpy(pairToPush->second, &m_ibeoScan, sizeof(SibeoScanData));
		  memcpy(pairToPush->second+sizeof(SibeoScanData)-sizeof(IbeoScanPoint*), m_ibeoScan.Point, sizeof(IbeoScanPoint)*numOfIbeoPoints);
		  vecToSend_mutex.lock();
		  vecToSend->insert(vecToSend->end(),pairToPush);
		  vecToSend_mutex.unlock();
		  delete m_ibeoScan.Point;
	  }

	}
    

    //littleEndianToBig
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

    //BroadcastTF
    void BroadcastTF(ros::Time new_scan_time_tag)
    {
      static tf::TransformBroadcaster br;
      br.sendTransform(tf::StampedTransform(_laser_TF_point_of_origin, new_scan_time_tag, _TF_parent_link_name, _Sensor_Name+"_laser_point_of_origin"));
    }

    //fillPointsForRVIZ
    void fillPointsForRVIZ(visualization_msgs::Marker& points, float ang_pitch, float factor_of_ang_yaw, vector<double> ranges)
    {
        for (unsigned int i = 0; i < ranges.size(); ++i)
        {
          float ang_yaw = factor_of_ang_yaw * i;
          float R = ranges[i];

          geometry_msgs::Point p;
          p.x = R*cos(ang_yaw)*cos(ang_pitch);
          p.y = R*sin(ang_yaw);
          p.z = -R*cos(ang_yaw)*sin(ang_pitch);

          points.points.push_back(p);
        }
    }

    //Publish_RVIZ_Message
    void Publish_RVIZ_Message(vector<double>& rangesT1, vector<double>& rangesT2, vector<double>& rangesB1, vector<double>& rangesB2 , ros::Time new_scan_time_tag)
	{

	visualization_msgs::Marker points;
	points.header.frame_id = _TF_parent_link_name; // "/world";
	points.header.stamp = new_scan_time_tag;
	points.ns = _Robot_Name_Space+"/"+_Sensor_Name;
	points.action = visualization_msgs::Marker::ADD;
	points.pose.orientation.w = 1.0;

	points.id = rviz_points_index++ ;
	points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;  // POINTS markers use x and y scale for width/height respectively
    points.scale.y = 0.05;

    points.color.g = 1.0f;  // Points are green
    points.color.a = 1.0;

    points.lifetime = ros::Duration(60,0);

	fillPointsForRVIZ(points, _row_t1_pitch_ang, _rows_t_end_ang + _rows_yaw_ang_increment, rangesT1);
	fillPointsForRVIZ(points, _row_t2_pitch_ang, _rows_t_end_ang + _rows_yaw_ang_increment, rangesT2);
	fillPointsForRVIZ(points, _row_b1_pitch_ang, _rows_b_end_ang + _rows_yaw_ang_increment, rangesB1);
	fillPointsForRVIZ(points, _row_b2_pitch_ang, _rows_b_end_ang + _rows_yaw_ang_increment, rangesB2);

    _marker_pub.publish(points);
    }

  private:
    physics::ModelPtr 			_model; // Pointer to the model
    event::ConnectionPtr 		_updateConnection; // Pointer to the update event connection
    common::Time			LastSensorUpdateTime;
    common::Time			sim_Time; // used for timing debug

    bool flag_fillMsg;

#ifdef USE_GPU
    sensors::GpuRaySensorPtr 		_sensorT1, _sensorT2, _sensorB1, _sensorB2;
#else
    sensors::RaySensorPtr 		_sensorT1, _sensorT2, _sensorB1, _sensorB2;
#endif

    ros::NodeHandle		_nodeHandle;
    ros::Publisher 		_publisher;
    ros::Publisher _marker_pub;
    int rviz_points_index;
    std::string _Robot_Name_Space;
    std::string _Sensor_Name;

    std::string _sensor_t1_name;
    std::string _sensor_t2_name;
    std::string _sensor_b1_name;
    std::string _sensor_b2_name;

    float _row_t1_pitch_ang;
    float _row_t2_pitch_ang;
    float _row_b1_pitch_ang;
    float _row_b2_pitch_ang;

    float _rows_t_end_ang;
    float _rows_t_start_ang;
    float _rows_b_end_ang;
    float _rows_b_start_ang;

    float _rows_yaw_ang_increment;

    double _scanning_frequency; //in Hz
    int numOfIbeoPoints;
    float _distance_sample_resolution;
    float _distance_min;
    float _distance_max;

    std::string _TF_parent_link_name;
    tf::Transform _laser_TF_point_of_origin;

    SibeoScanData m_ibeoScan;
    int sockfd,newsockfd,portno;
    socklen_t clilen;
    struct sockaddr_in serv_addr,cli_addr;
    struct pollfd ufds[2];
    std::atomic<bool> client_connected;
//    std::atomic<bool> initMsgWasSend;

    boost::thread _tcpSenderThread;
    boost::thread _tcpRestartThread;
    boost::thread _controlThread;

    boost::mutex vecToSend_mutex;
    std::vector<std::pair<common::Time, char[10000]> *> *vecToSend;

    double Delay;

    double FC_sumOfFrequencyForAverage;
    int FC_counterOfSecondsForFrequencyAverage;
    int FC_counterOfMsgInSec;
    int FC_LastTime_Second;
  };

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(IBEO)
}

#endif
