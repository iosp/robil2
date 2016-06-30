#define CENTER_X_NS		32.0111448 //31.2622		// GPS coordinates
#define CENTER_Y_EW		34.91200118 //34.803611	// of lab 320
#define DEGREE_TO_M		111000 			//1 degree has appprox. 111km
#define ABS(x) (x > 0 ? x : -x)


#define TOPIC_NAME_GPS 		"/SENSORS/GPS"
#define TOPIC_NAME_SPEED	"/SENSORS/GPS/Speed"
#define TOPIC_NAME_IMU 		"/SENSORS/IMU"
#define SENSOR_GPS_NAME		"gps_component"
#define SENSOR_IMU_NAME		"imu_component"

//#define UDP_SERVER			"132.4.6.90"
#define UDP_PORT_IPON 		"2010"

#define TCP_PORT_IPON 		2001

const double PI  =3.141592653589793238463;

#define Rad2Mills 6400/(2*PI)
#define Deg2Pi 1/180
#define TIME_INTERVAL_100HZ 0.01
#define TIME_INTERVAL_1HZ 1

typedef struct{
	float latitude;
	float longitude;
	float altitude;
}LatLonAlt;

typedef enum{
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_PERIODIC_100HZ = 1,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_PERIODIC_1HZ = 2,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_LEVER_ARM_AND_BORESIGHT = 3,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_SOFTWARE_VERSION_AND_SYSTEM_DATA = 4,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_LAYOUT_DATA_SETUP = 5,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_BIT_DATA = 6,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_ODOMETER_LEVER_ARM = 7,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_ODOMETER_CALIBRATION_DATA = 8,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_DATUM_TABLE = 9,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_REQUEST_DELAY_TABLE = 10,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_RESERVED1 = 11,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_RESERVED2_0001 = 12,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_RESERVED3_0002 = 13,
	E_MESSAGE_ID_ACCEPTED_FROM_EGI_REPLAY_FOR_KEEP_ALIVE = 14
}E_MESSAGE_ID_ACCEPTED_FROM_EGI;

typedef struct{
	unsigned char GPS_Data_Not_Valid_Egi;
	unsigned char Temperature_Fail_Egi;
	unsigned char NA_Egi;
	unsigned char NA1_Egi;
	unsigned char NA2_Egi;
	unsigned char NA3_Egi;
	unsigned char NA4_Egi;
	unsigned char NA5_Egi;
	unsigned char Severe_INS_Fail_Egi;
	unsigned char External_Voltage_Fail_Egi;
	unsigned char Antena_Fail_Egi;
	unsigned char ONE_PPS_Fail_Egi;
	unsigned char IMU_Fail_Egi;
	unsigned char GPS_Fail_Egi;
	unsigned char ODO_Fail_Egi;
	unsigned char INS_Fail_Egi;
}VALIDITY_WORD_EGI_STRUCT;

typedef char EIGHT_SPARE_BYTE_ARRAY[8];

typedef struct{
	unsigned short SOM;
	E_MESSAGE_ID_ACCEPTED_FROM_EGI Message_ID_Accepted_From_EGI;
	unsigned short blockLength;
	double INS_Time_Of_Nav_Data;
	double LAT_Egi; //range: [-1.0, 1.0]
	double LONG_Egi; //range: [-1.0, 1.0]
	double Altitude_MSL_EGI;
	double Pitch_PD_Egi;
	double Roll_PD_Egi;
	double Azimuth_PD_geographic;
	double Roll_rate_X_PD_Egi;
	double Pitch_rate_Y_PD_Egi;
	double Azimuth_rate_Z_PD_Egi;
	double ECC_X_Egi;
	double ECC_Y_Egi;
	double ECC_Z_Egi;
	double Velocity_north_Egi;
	double Velocity_East_Egi;
	double Velocity_down_Egi;
	double Alt_correction_Egi;
	VALIDITY_WORD_EGI_STRUCT Validity_Word;
	unsigned short Input_Message_Number_Echo_Egi;
	EIGHT_SPARE_BYTE_ARRAY Eight_byte_Spare;
	unsigned short Checksum_Egi;
}PHSPERIODIC100HZMESSAGE;


/* S2_STATUS_STRUCT */
typedef struct {
	unsigned char                                   Zero_Velocity_update_in_progress; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   INS_or_ODO_or_GPS_malfunction; //
	unsigned char                                   Travel_Lock_status; //
	unsigned char                                   NA1_Egi; //
	unsigned char                                   INS_in_SH_align_mode; //
	unsigned char                                   INS_alert; //
	unsigned char                                   INS_in_align_on_the_move; //
} S2_STATUS_STRUCT;

/* S1_STATUS_STRUCT */
typedef struct {
	unsigned char                                   INS_in_startup_mode; //
	unsigned char                                   Startup_Complete; //
	unsigned char                                   INS_normal_or_SH_or_move_align; //
	unsigned char                                   INS_in_survey_mode; // set when INS enters survey mode
	unsigned char                                   INS_in_Exclusive_ZUPT_mode; //
	unsigned char                                   Zero_velocity_stop_request; // when set ,INS determines it needs a zero velocity stop
	unsigned char                                   Position_update_request; // when set, INS determines it needs a position update
	unsigned char                                   INS_ready_for_Align_on_the_move; //
} S1_STATUS_STRUCT;

/* S4_STATUS_STRUCT */
typedef struct {
	unsigned char                                   SH_Shutdown_test_completed; //
	unsigned char                                   Integrated_mode_of_operation; //
	unsigned char                                   INS_SH_Shutdown_failed; //
	unsigned char                                   INS_Shutdown_complete_successful; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   INS_in_standby_mode; //
	unsigned char                                   INS_wait_for_gps; //
	unsigned char                                   NA1_Egi; //
} S4_STATUS_STRUCT;

/* S3_STATUS_STRUCT */
typedef struct {
	unsigned char                                   Odo_calibration_in_process; //
	unsigned char                                   Odo_damping_in_process; //
	unsigned char                                   Odo_calibration_completed; //
	unsigned char                                   NA_Egi; //
	unsigned char                                   NA1_Egi; //
	unsigned char                                   INS_in_motion; //
	unsigned char                                   Orientation_attitude_data_valid; //
	unsigned char                                   Degraded_Survey; //
} S3_STATUS_STRUCT;

/* ALERT_DATA_D2_STRUCT */
typedef struct {
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   SH_Attitude_not_good; //
	unsigned char                                   Unable_to_complete_align; //
	unsigned char                                   Align_interrupt; //
	unsigned char                                   Bit_Reserved2; //
	unsigned char                                   Zero_Velocity_update_interrupt; //
	unsigned char                                   Bit_Reserved3; //
} ALERT_DATA_D2_STRUCT;

/* ALERT_WORD_1_STRUCT */
typedef struct {
	ALERT_DATA_D2_STRUCT                            Alert_D2; //
} ALERT_WORD_1_STRUCT;


/* ALERT_DATA_D3_STRUCT */
typedef struct {
	unsigned char                                   Gyro_Y_Anode_Two_Temperature; // Gyro Y anode two temperature is out of range
	unsigned char                                   Gyro_Z_Anode_One_Temperature; // Gyro Z anode one temperature is out of range
	unsigned char                                   Gyro_Z_Anode_Two_Temperature; // Gyro Z anode two temperature is out of range
	unsigned char                                   Gyro_X_Dither_Temperature; // Gyro X dither temperature is out of range
	unsigned char                                   Gyro_X_Cathode_Temperature; // Gyro X cathode temperature is out of range
	unsigned char                                   Gyro_Y_Dither_Temperature; // Gyro Y dither temperature is out of range
	unsigned char                                   Gyro_Y_Cathode_Temperature; // Gyro Y cathode temperature is out of range
	unsigned char                                   Gyro_Z_Dither_Temperature; // Gyro Z dither temperature is out of range
} ALERT_DATA_D3_STRUCT;

/* ALERT_DATA_D4_STRUCT */
typedef struct {
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   GPS_position_conflict; //
	unsigned char                                   Altitude_update_rejected; //
	unsigned char                                   Horizontal_PositionUpdate_reject; //
	unsigned char                                   BITS_3_Reserved; //
} ALERT_DATA_D4_STRUCT;

/* ALERT_WORD_2_STRUCT */
typedef struct {
	ALERT_DATA_D3_STRUCT                            Alert_D3; //
	ALERT_DATA_D4_STRUCT                            Alert_D4; //
} ALERT_WORD_2_STRUCT;

/* ALERT_DATA_D5_STRUCT */
typedef struct {
	unsigned char                                   Invalid_update_or_data_request; //
	unsigned char                                   Invalid_mode_request; //
	unsigned char                                   BITS_3_Reserved; //
	unsigned char                                   Invalid_data_received; //
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
} ALERT_DATA_D5_STRUCT;

/* ALERT_DATA_D6_STRUCT */
typedef struct {
	unsigned char                                   BITS_3_Reserved; //
	unsigned char                                   GPS_data_unusable; //
	unsigned char                                   Bit_Reserved; //
	unsigned char                                   Bit_Reserved1; //
	unsigned char                                   Bit_Reserved2; //
	unsigned char                                   ODO_data_unusable; //
} ALERT_DATA_D6_STRUCT;

/* ALERT_WORD_3_STRUCT */
typedef struct {
	ALERT_DATA_D5_STRUCT                            Alert_D5; //
	ALERT_DATA_D6_STRUCT                            Alert_D6; //
} ALERT_WORD_3_STRUCT;

typedef enum {
	E_FOM_SMALLER_THAN_25M                         = 1,
	E_FOM_BETWEEN_25M_AND_50M                      = 2,
	E_FOM_BETWEEN_50M_AND_75M                      = 3,
	E_FOM_BETWEEN_75M_AND_100M                     = 4,
	E_FOM_BETWEEN_100M_AND_200M                    = 5,
	E_FOM_BETWEEN_200M_AND_500M                    = 6,
	E_FOM_BETWEEN_500M_AND_1000M                   = 7,
	E_FOM_BETWEEN_1000M_AND_5000M                  = 8,
	E_FOM_BIGGER_THAN_5000M                        = 9,
	E_FOM_DISCONNECTED_OR_NOT_READY                = 10
}E_FOM;

typedef struct{
	S2_STATUS_STRUCT s2_status;
	S1_STATUS_STRUCT s1_status;
	S4_STATUS_STRUCT s4_status;
	S3_STATUS_STRUCT s3_status;
}STATUS_WORD_STRUCT;

typedef struct{
	unsigned short SOM;
	E_MESSAGE_ID_ACCEPTED_FROM_EGI Message_ID_Accepted_From_EGI;
	unsigned short blockLength;
	double INS_Time_Of_Nav_Data;
	double GPS_TINE_EGI;
	unsigned long INS_DISTANCE_TRAVELED;
	double GPS_LAT_Egi;
	double GPS_LONG_Egi;
	double GPS_Altitude_Egi;
	unsigned short Alignment_Countdown;
	STATUS_WORD_STRUCT Status_word;
	ALERT_WORD_1_STRUCT Alert_word_1;
	ALERT_WORD_2_STRUCT Alert_word_2;
	ALERT_WORD_3_STRUCT Alert_word_3;
	double Azimuth_Error_RMS;
	double Velocity_error_RMS;
	unsigned short INS_Horizontal_Position_Error;
	unsigned short INS_Altitude_Error;
	double Roll_Error_RMS;
	double Pitch_Error_RMS;
	unsigned char Number_Of_Satellites;
	E_FOM Figure_Of_Merit;
	unsigned short week;
	unsigned short GPS_UTC_offset;
	unsigned short Bytes_2_reserved;
	unsigned short Checksum_Egi;
}PHSPERIODIC1HZMESSAGE;

//end Akiva
