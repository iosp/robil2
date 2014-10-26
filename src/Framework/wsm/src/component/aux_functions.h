
double glSpeedloc(double speedx,double speedy, double yaw)
{
	double theta = atan2(speedy,speedx);
	double eps = 3.14 / 4;
	if (yaw > theta - eps && theta + eps > yaw)
		return sqrt(speedx*speedx + speedy*speedy);
	return -sqrt(speedx*speedx + speedy*speedy);
}



