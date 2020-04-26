

#pragma once


#include "opencv2/opencv.hpp"
#include <vector>
namespace DD {

struct CloudPoint {
	cv::Point3d pt;
	std::vector<int> imgpt_for_img;
	double reprojection_error;
};

}
