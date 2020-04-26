// Seung-Chan Kim
// ver 0.1


#pragma once


#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <iostream>

using namespace cv;

namespace DD {


	//////

	static void KeyPointsToPoints(const vector<KeyPoint>& kps, vector<Point2f>& ps) {
		ps.clear();
		for (unsigned int i = 0; i<kps.size(); i++) ps.push_back(kps[i].pt);
	}

	static vector<Point2f> KeyPointsToPoints(const vector<KeyPoint>& kps) {
		vector<Point2f> ps;
		for (unsigned int i = 0; i<kps.size(); i++) ps.push_back(kps[i].pt);
		return ps;

	}


	static void PointsToKeyPoints(const vector<Point2f>& ps, vector<KeyPoint>& kps) {
		kps.clear();
		for (unsigned int i = 0; i<ps.size(); i++) kps.push_back(KeyPoint(ps[i], 1.0f));
	}



	template<typename T>
	vector<Point_<T> > Points(const vector<KeyPoint>& keypoints)
	{
		vector<Point_<T> > res;
		for (unsigned i = 0; i < keypoints.size(); i++) {
			res.push_back(Point_<T>(keypoints[i].pt.x, keypoints[i].pt.y));
		}
		return res;
	}

	template<typename T>
	vector<KeyPoint> KeyPoints(const vector<Point_<T> >& points) {
		vector<KeyPoint> res;
		for (unsigned i = 0; i < points.size(); i++) {
			res.push_back(KeyPoint(points[i], 1, 0, 0));
		}
		return res;
	}

}
