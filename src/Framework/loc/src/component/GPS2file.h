
#ifndef GPS2FILE_H_
#define GPS2FILE_H_

#include <ctime>
#include <fstream>


inline const std::string currentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    // Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
    // for more information about date/time format
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}

inline void gps2file(double altitude,double latitude,double longitude)
{
	std::cout << "Saving GPS coordinates to file\n";
	std::ofstream f ("gps_init.txt");
	if (f.is_open())
	{
	  f << "Time " << currentDateTime() << "\n";
	  f << "altitude " << std::setprecision(9) << altitude << "\n";
	  f << "latitude " << std::setprecision(9) << latitude << "\n";
	  f << "longitude " << std::setprecision(9) << longitude << "\n";
	  f.close();
	  std::cout << "created file with initial GPS values.\n";
	}
	else std::cout << "Unable to create GPS file";
}
#endif