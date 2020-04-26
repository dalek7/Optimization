// Seung-Chan Kim
// ver 0.1


#ifndef __CVUTILCALCHELPER_H__
#define __CVUTILCALCHELPER_H__


#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <iostream>

using namespace cv;

namespace DD {

	///////////////////////////////////
	// Operators

	//http://opencv-users.1802565.n2.nabble.com/How-to-multiply-Matrix-and-point-tp5270927p7296722.html
	static Point2f operator*(Mat M, const Point2f& p){
		Mat src(3/*rows*/, 1 /* cols */, CV_64F);

		src.at<double>(0, 0) = p.x;
		src.at<double>(1, 0) = p.y;
		src.at<double>(2, 0) = 1.0;

		Mat dst = M*src; //USE MATRIX ALGEBRA
		return Point2f(dst.at<double>(0, 0), dst.at<double>(1, 0));
	}


	static bool operator==(const Point3d& p1, const Point3d& p2){
		if( (p1.x == p2.x ) && (p1.y == p2.y )&& (p1.z == p2.z ))	//check this !!
			return true;
		else
			return false;
	}

    static bool operator==(const Point2f& p1, const Point2f& p2){
            if( (p1.x == p2.x ) && (p1.y == p2.y ))	//check this !!
                return true;
            else
                return false;
    }


    static bool operator==(const KeyPoint& p1, const KeyPoint& p2){
            if( (p1.pt.x == p2.pt.x ) && (p1.pt.y == p2.pt.y ))	//check this !!
                return true;
            else
                return false;
    }


	static Point2f operator+(const Point2f& p1, const Point2f& p2){
		Point2f sum;
		sum.x = p1.x + p2.x;
		sum.y = p1.y + p2.y;

		return sum;
	}

	static Point2d operator+(const Point2d& p1, const Point2d& p2){
		Point2d sum;
		sum.x = p1.x + p2.x;
		sum.y = p1.y + p2.y;

		return sum;
	}
	static Point2d operator*(Mat M, const Point2d& p){
			Mat src(3/*rows*/,1 /* cols */,CV_64F);

			src.at<double>(0,0)=p.x;
			src.at<double>(1,0)=p.y;
			src.at<double>(2,0)=1.0;

			Mat dst = M*src; //USE MATRIX ALGEBRA
			return Point2d(dst.at<double>(0,0),dst.at<double>(1,0));
	}


	static Point3d operator*(Mat M, const Point3d& p){
			Mat src(4/*rows*/,1 /* cols */,CV_64F);

			src.at<double>(0,0)=p.x;
			src.at<double>(1,0)=p.y;
			src.at<double>(2,0)=p.z;
			src.at<double>(3,0)=1.0;

			Mat dst = M*src; //USE MATRIX ALGEBRA
			return Point3d(dst.at<double>(0,0),dst.at<double>(1,0), dst.at<double>(2,0));
	}




	///////////////////////////////////
	// Matches


	static std::vector<cv::DMatch> FlipMatches(const std::vector<cv::DMatch>& matches) {
		std::vector<cv::DMatch> flip;
		for(unsigned int i=0;i<matches.size();i++) {
			flip.push_back(matches[i]);
			swap(flip.back().queryIdx,flip.back().trainIdx);
		}
		return flip;
	}




	static void GetAlignedPointsFromMatch(const std::vector<cv::KeyPoint>& imgpts1,
								   const std::vector<cv::KeyPoint>& imgpts2,
								   const std::vector<cv::DMatch>& matches,
								   std::vector<cv::KeyPoint>& pt_set1,
								   std::vector<cv::KeyPoint>& pt_set2)
	{
		for (unsigned int i=0; i<matches.size(); i++) {
	//		cout << "matches[i].queryIdx " << matches[i].queryIdx << " matches[i].trainIdx " << matches[i].trainIdx << endl;


			pt_set1.push_back(imgpts1[matches[i].queryIdx]);
			pt_set2.push_back(imgpts2[matches[i].trainIdx]);
		}
	}




	static void MakeHusingRandt(Mat_<double> R1, Mat_<double> t1, Mat& P)
	{

		/*
		Mat_<double> R2(3,3);
		Mat_<double> t1(1,3);

		*/
		Mat_<double> P1 = (Mat_<double>(3, 4) <<
			R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
			R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
			R1(2, 0), R1(2, 1), R1(2, 2), t1(2));


		P = P1;
	}



	static Mat_<double> MakeHusingRandt(Mat_<double> R1, Mat_<double> t1)
	{

		/*
		Mat_<double> R2(3,3);
		Mat_<double> t1(1,3);

		*/
		Mat_<double> P1 = (Mat_<double>(3, 4) <<
			R1(0, 0), R1(0, 1), R1(0, 2), t1(0),
			R1(1, 0), R1(1, 1), R1(1, 2), t1(1),
			R1(2, 0), R1(2, 1), R1(2, 2), t1(2));


		return P1;
	}






	template <typename T>
	static void DecomposeH2Randt(Mat& P, Mat &R1, Mat &t1)
	{

		/*
		Mat_<double> R2(3,3);
		Mat_<double> t1(1,3);

		*/
		Mat_<T> t = P.col(3);
		t1 = t;

		cv::Rect ROI(0, 0, 3, 3);
		Mat_<T> R = P(ROI).clone();

		R1 = R;

		/*
		cout <<P<<endl<<endl;
		cout <<t1 <<endl;
		cout <<R1 <<endl;
		*/

	}




	static int GetHeightFromWithWidth(const Mat img, const int w_target)
	{

		int w = img.size().width;
		int h = img.size().height;

		int h_target = h * w_target / (float)w;

		return h_target;
	}



	template <typename T>
	static bool Normalize4D(const Mat& points4D, Mat& points3D)
	{
		Mat mat = points4D.t();
		int i, j;
		for (i = 0; i < mat.rows; i++)
		{

			double x = mat.at<float>(i, 0) / mat.at<float>(i, 3);
			double y = mat.at<float>(i, 1) / mat.at<float>(i, 3);
			double z = mat.at<float>(i, 2) / mat.at<float>(i, 3);

			T pt(x, y, z);
			points3D.push_back(pt);
		}

		return 1;
	}
	// --> TO TEST !!
	static void GetGoodKeypoints(	const vector< DMatch > good_matches, const vector<KeyPoint> keypoints1,const  vector<KeyPoint> keypoints2,
											vector<KeyPoint>& keypoints1_out,vector<KeyPoint>& keypoints2_out)
	{

		vector<int> pointIndexes1;
		vector<int> pointIndexes2;

		vector<KeyPoint> keypoints1o;
		vector<KeyPoint> keypoints2o;

		for (vector<cv::DMatch>::const_iterator it= good_matches.begin(); it!= good_matches.end(); ++it) {

				 // Get the indexes of the selected matched keypoints
				 pointIndexes1.push_back(it->queryIdx);
				 pointIndexes2.push_back(it->trainIdx);
		}


		for(unsigned int i = 0; i < pointIndexes1.size(); i++ )
		{
			int idx = pointIndexes1[i];
			if( idx >= 0 )
				keypoints1o.push_back(keypoints1[idx]);

		}

		for(unsigned int i = 0; i < pointIndexes2.size(); i++ )
		{
			int idx = pointIndexes2[i];
			if( idx >= 0 )
				keypoints2o.push_back(keypoints2[idx]);

		}

		keypoints1_out = keypoints1o;
		keypoints2_out = keypoints2o;
	}

	static void ConvertKeypointsIntoPoint2f(vector< DMatch > good_matches, vector<KeyPoint> keypoints1,vector<KeyPoint> keypoints2, vector<cv::Point2f>& selPoints1, vector<cv::Point2f>& selPoints2)
	{
		// Convert 1 vector of keypoints into 2 vectors of Point2f
		vector<int> pointIndexes1;
		vector<int> pointIndexes2;
		for (vector<cv::DMatch>::const_iterator it= good_matches.begin(); it!= good_matches.end(); ++it) {

				 // Get the indexes of the selected matched keypoints
				 pointIndexes1.push_back(it->queryIdx);
				 pointIndexes2.push_back(it->trainIdx);
		}

		//// Convert keypoints into Point2f
		//vector<cv::Point2f> selPoints1, selPoints2;
		cv::KeyPoint::convert(keypoints1,selPoints1,pointIndexes1);
		cv::KeyPoint::convert(keypoints2,selPoints2,pointIndexes2);
	}



	template<typename T, typename V>
	void keepVectorsByStatus(vector<T>& f1, vector<V>& f2, const vector<uchar>& status) {
		vector<T> oldf1 = f1;
		vector<V> oldf2 = f2;
		f1.clear();
		f2.clear();
		for (unsigned int i = 0; i < status.size(); ++i) {
			if (status[i])
			{
				f1.push_back(oldf1[i]);
				f2.push_back(oldf2[i]);
			}
		}
	}



    template<typename T, typename V>
	static void keepVectorsByStatus2(vector<T>& f0, vector<T>& f1, vector<V>& f2, const vector<uchar>& status) {
		vector<T> oldf0 = f0;
		vector<T> oldf1 = f1;
		vector<V> oldf2 = f2;
		f0.clear();
		f1.clear();
		f2.clear();
		for (int i = 0; i < status.size(); ++i) {
			if (status[i])
			{
				f0.push_back(oldf0[i]);
				f1.push_back(oldf1[i]);
				f2.push_back(oldf2[i]);
			}
		}
	}



	static bool CheckCoherentRotation(cv::Mat_<double>& R) {
		//std::cout << "R; " << R << std::endl;
		//double s = cv::norm(cv::abs(R),cv::Mat_<double>::eye(3,3),cv::NORM_L1);
		//std::cout << "Distance from I: " << s << std::endl;
		//if (s > 2.3) { // norm of R from I is large -> probably bad rotation
		//	std::cout << "rotation is probably not coherent.." << std::endl;
		//	return false;	//skip triangulation
		//}
		//Eigen::Map<Eigen::Matrix<double,3,3,Eigen::RowMajor> > eR(R[0]);
		//if(eR(2,0) < -0.9)
		//{
		//	cout << "rotate 180deg (PI rad) on Y" << endl;

		//	cout << "before" << endl << eR << endl;
		//	Eigen::AngleAxisd aad(-M_PI/2.0,Eigen::Vector3d::UnitY());
		//	eR *= aad.toRotationMatrix();
		//	cout << "after" << endl << eR << endl;
		//}
		//if(eR(0,0) < -0.9) {
		//	cout << "flip right vector" << endl;
		//	eR.row(0) = -eR.row(0);
		//}

		if(fabsf(determinant(R))-1.0 > 1e-07) {
			cerr << "det(R) != +-1.0, this is not a rotation matrix" << endl;
			return false;
		}

		return true;
	}


	static void checkSVD(const Mat& pt_3d, float* w)
	{
		int ncloud	= pt_3d.rows;
//		int ncol	= pt_3d.cols;

		cv::Mat temp;
		pt_3d.copyTo(temp);

		cv::Point3f sum;
		for(int i=0; i<ncloud; i++)
		{
			sum.x += temp.at<float>(0,i);
			sum.y += temp.at<float>(1,i);
			sum.z += temp.at<float>(2,i);

		}

		sum.x /= (float)ncloud;
		sum.y /= (float)ncloud;
		sum.z /= (float)ncloud;

		for(int i=0; i<ncloud; i++)
		{

			temp.at<float>(0,i) -= sum.x;
			temp.at<float>(1,i) -= sum.y;
			temp.at<float>(2,i) -= sum.z;
		}


		cv::SVD svd(pt_3d);

		//cout << svd.w.size(); // 1x3
		//double sv_ratio = fabsf(svd.w.at<float>(2) / svd.w.at<float>(1));

		if(w)
		{
			w[0] = svd.w.at<float>(0);
			w[1] = svd.w.at<float>(1);
			w[2] = svd.w.at<float>(2);


		}

		//printf("sv ratio = %f\n", sv_ratio);
	}




	template <typename T>
	static void GetWfromSVD(const std::vector<T>& cloud, float* w_out)
	{

		// W vectoer from SVD
		Mat pt_3d(3,cloud.size(),CV_32FC1);
		int i = 0;
		for(typename std::vector<T>::const_iterator it = cloud.begin(); it!= cloud.end(); ++it, ++i)
		{

			pt_3d.at<float>(0, i) =  it->pt.x;
			pt_3d.at<float>(1, i) =  it->pt.y;
			pt_3d.at<float>(2, i) =  it->pt.z;

		}

		//float w[3];
		if(w_out)
		checkSVD(pt_3d, w_out);


		//checkSVDofCloud2(pt_3d, w);





	}




	template<typename T>
	vector<T> RemoveMean2D(const vector<T> cloud, float& mean_x, float& mean_y)
	{
		typename vector<T>::const_iterator itf = cloud.begin();

		int n_pt = cloud.size();

		Point2d ptm;
		for(; itf!= cloud.end(); ++itf)
		{
			ptm.x += itf->x;
			ptm.y += itf->y;

		}

		ptm.x /= (double) n_pt;
		ptm.y /= (double) n_pt;

		mean_x = ptm.x;
		mean_y = ptm.y;

		itf = cloud.begin();
		vector<T> selPoints1c;

		for(; itf!= cloud.end(); ++itf)
		{
			cv::Point2d pt1;
			pt1.x = itf->x - ptm.x;
			pt1.y = itf->y - ptm.y;

			selPoints1c.push_back(pt1);
		}

		//	cout << "features' mean (" << ptm.x <<" , " << ptm.y <<" )" <<endl;


		return selPoints1c;

	}



	// X-Y-Z fixed angles
	// R_XYZ*(r, b, a)
	// actual : r about X_A --> b about Y_A --> a about Z_A

	template <typename T, typename V>
	static Point3d GetRPY(const Mat& R)
	{
		//RPY( a,b,c )	= Rot(z,a)Rot(y,b)Rot(x,c)
		/*
			[Cos(a)*Cos(b), -(Cos(c)*Sin(a)) + Cos(a)*Sin(b)*Sin(c), Cos(a)*Cos(c)*Sin(b) + Sin(a)*Sin(c)),
			[Cos(b)*Sin(a), Cos(a)*Cos(c) + Sin(a)*Sin(b)*Sin(c)   , Cos(c)*Sin(a)*Sin(b) - Cos(a)*Sin(c)),
			[-Sin(b)      , Cos(b)*Sin(c)                          , Cos(b)*Cos(c)
		*/

		/*
		Mat_<double> P1 = (Mat_<double>(3,4) <<
							R1(0,0),   R1(0,1),    R1(0,2),    t1(0),
							R1(1,0),   R1(1,1),    R1(1,2),    t1(1),
							R1(2,0),   R1(2,1),    R1(2,2),    t1(2));

		*/
		// column-wise

		float v[16] = {	R.at<T>(0,0), R.at<T>(1,0), R.at<T>(2,0), 0,
						R.at<T>(0,1), R.at<T>(1,1), R.at<T>(2,1), 0,
						R.at<T>(0,2), R.at<T>(1,2), R.at<T>(2,2), 0,
							0,0,0,1};
		/*
		0 4 8  12
		1 5 9  13
		2 6 10 14
		3 7 11 15

		0 3 6
		1 4 7
		2 5 8

		*/

		float a,b,r,cb;	// from P.47 of J.Craig.
		b	= atan2( -v[2],sqrt( v[0]*v[0] + v[1]*v[1]) );
		cb	= cos(b);


		a	= atan2( v[1]/cb,v[0]/cb);
		r	= atan2( v[6]/cb,v[10]/cb);

		// need to consider the degenerate cases, cb = 0

		V out  = V(a,b,r);


		return out;
	}



}

#endif
