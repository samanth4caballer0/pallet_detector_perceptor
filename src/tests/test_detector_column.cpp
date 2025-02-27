#include "target_detector/detector_column.h"

int main(int argc, char **argv)
{
	std::cout << "Test Detector Column" << std::endl;

	TargetDetector::DetectorColumn detector__;
	std::map<std::string, double> detector_params__;

	// configure
	detector_params__["column_size"] = 0.5;
	detector_params__["max_column_range"] = 25;
	detector__.configure(detector_params__);

	// synthetic scan data
/*	double angle_init = -M_PI;
	double angle_end = M_PI;
	std::vector<float> ranges(3600);
	std::vector<float> intensities(3600);
	std::vector<double> detections;
	for ( unsigned int ii=0; ii<ranges.size(); ii++ )
	{
		ranges[ii] = 19;
		intensities[ii] = 200;
		if ( ( ii >895 ) && ( ii < 905 ) ) // -pi/2
		{
			ranges[ii] = 7.9;
			intensities[ii] = 1000;
		}
		if ( ( ii >2695 ) && ( ii < 2705 ) ) // pi/2
		{
			ranges[ii] = 8.2;
			intensities[ii] = 800;
		}

	}

	// detect reflectors at lidar frame, and print
	Eigen::Isometry2d T_platform_sensor;
	T_platform_sensor.matrix() = Eigen::Matrix3d::Identity();
	detector__.detect(angle_init, angle_end, ranges, intensities, T_platform_sensor, detections);
	for ( unsigned int ii=0; ii<detections.size(); ii++ )
	{
		std::cout << detections[ii] << " ";
		if ( (ii+1)%6 == 0 ) std::cout << std::endl;
	}

	// detect reflectors at platform frame, and print
	detections.clear();
	T_platform_sensor.matrix() <<
		std::cos(M_PI/4), -std::sin(M_PI/4), 1,
		std::sin(M_PI/4),  std::sin(M_PI/4), 0.5,
		0,0,1;
	detector__.detect(angle_init, angle_end, ranges, intensities, T_platform_sensor, detections);
	std::cout << std::endl;
	for ( unsigned int ii=0; ii<detections.size(); ii++ )
	{
		std::cout << detections[ii] << " ";
		if ( (ii+1)%6 == 0 ) std::cout << std::endl;
	}
*/
	// return
	return 0;
}
