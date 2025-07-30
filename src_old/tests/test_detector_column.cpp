#include "target_detector/detector_column.h"

int main(int argc, char **argv)
{
	std::cout << "Test Detector Column" << std::endl;

	TargetDetector::DetectorColumn detector__;
	std::map<std::string, double> detector_params__;

	// configure
	detector_params__["target_size"] = 0.5;
	detector_params__["max_target_range"] = 25;
	detector__.configure(detector_params__);

	// synthetic scan data
	double angle_init = -M_PI;
	double angle_end = M_PI;
	std::vector<float> ranges(3600);
	std::vector<float> intensities(3600);
	std::vector<double> detections;
	for ( unsigned int ii=0; ii<ranges.size(); ii++ )
	{
		ranges[ii] = 19;
		intensities[ii] = 0;
		if ( ( ii >880 ) && ( ii < 920 ) ) // -pi/2
		{
			ranges[ii] = 7.9;
		}
		if ( ( ii >2680 ) && ( ii < 2720 ) ) // pi/2
		{
			ranges[ii] = 8.2;
		}

	}

	// detect reflectors at lidar frame, and print
	std::cout << std::endl << "Columns in sensor frame" << std::endl;
	Eigen::Isometry2d T_platform_sensor;
	T_platform_sensor.matrix() = Eigen::Matrix3d::Identity();
	detector__.detect(angle_init, angle_end, ranges, intensities, T_platform_sensor, detections);
	for ( unsigned int ii=0; ii<detections.size(); ii++ )
	{
		std::cout << detections[ii] << " ";
		if ( (ii+1)%6 == 0 ) std::cout << std::endl;
	}

	// detect reflectors at platform frame, and print
	std::cout << std::endl << "Columns in platform frame" << std::endl;
	detections.clear();
	T_platform_sensor.matrix() <<
		std::cos(M_PI/4), -std::sin(M_PI/4), 1,
		std::sin(M_PI/4),  std::sin(M_PI/4), 0.5,
		0,0,1;
	detector__.detect(angle_init, angle_end, ranges, intensities, T_platform_sensor, detections);
	for ( unsigned int ii=0; ii<detections.size(); ii++ )
	{
		std::cout << detections[ii] << " ";
		if ( (ii+1)%6 == 0 ) std::cout << std::endl;
	}

	// return
	return 0;
}
