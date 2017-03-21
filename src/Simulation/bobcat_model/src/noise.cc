#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <random>
#include <cmath>
//#include <mutex>

class noise
{
public:

	noise()
	{
		currentNoise = 0;
		last_simTime = 0;
		initSuccessed = false;
        srand(time(NULL));
	}
	~noise()
	{
		delete Noise_dist;
	}

	void init(double frequencyInHz, double mu, double sigma)
	{
        /* initialize random seed: */

		//without mutex
		std::normal_distribution<double> * TEMP_P_FOR_DELETE = Noise_dist;
		Noise_dist = new std::normal_distribution<double>(mu, sigma);
		delete TEMP_P_FOR_DELETE;

		//mutex
//		mtx.lock();
//		delete Noise_dist;
//        Noise_dist = new std::normal_distribution<double>(mu, sigma);
//        mtx.unlock();


		if(frequencyInHz != 0)
		{
			TimeInterval = (double)((double)1.0/(double)frequencyInHz );
			initSuccessed = true;
		}
		else
			initSuccessed = false;

	}
	
	void onUpdate(const double currentSimTime)
	{
		if(!initSuccessed)
			return;

		double dt = currentSimTime-last_simTime;
		double diff_update_time = dt - TimeInterval;
		if (diff_update_time >= -0.0001)
		{
			last_simTime = currentSimTime;
			updaeteNoise();
		}
	}

	double getCurrentNoise()
	{
		return currentNoise;
	}

private:
	void updaeteNoise()
	{
//		mtx.lock();
		currentNoise = (*Noise_dist)(generator);
//		mtx.unlock();
	}

	double currentNoise;
	bool initSuccessed;
	double last_simTime;
	double TimeInterval;
    std::default_random_engine generator;
    std::normal_distribution<double> *Noise_dist;

//    std::mutex mtx;
};
