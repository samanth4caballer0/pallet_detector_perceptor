#include "detectors/detector_pcl_base.h"

// namespace
namespace Detectors
{

DetectorPclBase::DetectorPclBase()
{
	//
}

DetectorPclBase::~DetectorPclBase()
{
	//
}

void DetectorPclBase::crop(
	const Eigen::Vector4f & __max_values,
	const Eigen::Vector4f & __min_values,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out,
	bool __negative)
{
	pcl::CropBox<pcl::PointXYZ> box_filter;
	box_filter.setMax(__max_values);
	box_filter.setMin(__min_values);
	box_filter.setInputCloud(__cloud_in);
	box_filter.setNegative(__negative);
	box_filter.filter(*__cloud_out);
}

void DetectorPclBase::voxelDownsampling(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(__cloud_in);
    voxel_grid.setLeafSize(0.01f, 0.01f, 0.01f);  // 1cm x 1cm x 1cm voxels
    voxel_grid.filter(*__cloud_out);
}

void DetectorPclBase::removeOutliers(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out)
{
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> outlier_filter;
	outlier_filter.setMeanK(50);
	outlier_filter.setStddevMulThresh(0.1);
	outlier_filter.setInputCloud(__cloud_in);
	outlier_filter.filter(*__cloud_out);
}

void DetectorPclBase::computeNormals(
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in,
	pcl::PointCloud<pcl::Normal>::Ptr __cloud_out)
{
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_computer;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree(new pcl::search::KdTree<pcl::PointXYZ>());

	normal_computer.setInputCloud(__cloud_in);
	normal_computer.setSearchMethod(kd_tree);
	normal_computer.setRadiusSearch (0.05);
	normal_computer.compute(*__cloud_out);
}

unsigned int DetectorPclBase::correlation(
	const std::vector<double> & __data,
	const std::vector<double> & __kernel)
{
	//std::cout << __FILE__ << ":" << __LINE__ << std::endl;
	//std::cout << "-------------------------------------" << std::endl;
	double corr, sum;
	std::vector <double> corr_v;
	unsigned int half_kernel_size = (unsigned int)((__kernel.size()-1)/2);
	for(unsigned int ii=half_kernel_size; ii<(__data.size()-half_kernel_size); ii++ )
	{
		corr = 0;
		sum = 0;
		for (unsigned int jj=0; jj<__kernel.size(); jj++)
		{
			if ( __data[ii-half_kernel_size+jj] == 0.0 ) corr += -1000;
			else corr +=  __data[ii-half_kernel_size+jj] * __kernel[jj];
			//sum += __data[ii-half_kernel_size+jj];
		}
		//if (sum == 0.0) sum = 1.0;
		sum = 1.0;
		corr_v.push_back(corr/sum);
		//std::cout << "   " << ii << ": " << __data[ii] << " ; " << corr/sum << std::endl;
	}

	std::vector<double>::iterator it_max;
	it_max = std::max_element(corr_v.begin(), corr_v.end());
	unsigned int best_ii = std::distance(corr_v.begin(),it_max) + half_kernel_size;
	//std::cout << "Max element at " << best_ii << std::endl;

	//std::cout << "-------------------------------------" << std::endl;
	return best_ii;
}

bool DetectorPclBase::saveOnDisk(
	const std::string & __file_name,
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr __cloud_in)
{
	pcl::io::savePCDFileASCII (__file_name, *__cloud_in);
	return true;
}

bool DetectorPclBase::loadFromDisk(
	const std::string & __file_name,
	pcl::PointCloud<pcl::PointXYZ>::Ptr __cloud_out)
{
	if (pcl::io::loadPCDFile<pcl::PointXYZ> (__file_name, *__cloud_out) == -1)
	{
		std::cout << "Couldn't read file from disk at " << __file_name << std::endl;
		return false;
	}
	return true;
}

} //end of namespace
