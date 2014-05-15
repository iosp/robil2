/*
 * userHeader.h
 *
 *  Created on: May 11, 2014
 *      Author: lar5
 */

#ifndef USERHEADER_H_
#define USERHEADER_H_

/*
 * This file should be edited by the user in order to customize the localization process for the user needs.
 * Please read the comments carefully and read the Wiki for additional information.
 *
 * NOTE - Any change in this file requires a complete make of the workspace (catkin_make)
 */

#define _added_noise false //Is there noise added to the sensors
/* Added noise
 * You can check if there is noise added by going to ~/.gazebo/models/GPS_INS/model.sdf and check if the <noise> .... </noise> is on.
 * If you wish to work in a more realistic environment which consists of noise, make sure the <noise> tab is not commented and that the _added_noise variable above is set to true.
 *
 * If you wish to work in a noiseless environment and get a 100% accurate estimation of the location of the vehicle set _added_noise to false and make sure that the <noise> is commented.
 *
 *
 */

#define _init_latitude 31.2622 //_init_latitue - Recommended for debugging mode - end value of -1 to turn off
#define _init_longitude 34.803611 //_init_longitude - Recommended for debugging mode - end value of -1 to turn off
/* Initial position
 * In the case that noise is added, 100 reading of the GPS and IMU are taken to estimate the initial position of the vehicle for gazebo reference.
 * In any case the mean of the latitude and longitude received from the GPS will not be identical to the true location of the vehicle. This will lead to a constant bias error.
 *
 * For debugging purposes, it is recommended to override the initial location estimation and give the exact location of the vehicle and thus eliminating the bias error.
 *
 * The values of _init_latitude & _init_longitude is taken from  ~/.gazebo/models/GPS_INS/model.sdf from <init_latitue>....</init_latitue> and <init_longitude>... </init_longitude>.
 *
 * A value of -1 in either one of the variables will automatically stop this feature of initial position, and the mean of 100 GPS measurements will be set as the initial location.
 */



#endif /* USERHEADER_H_ */
