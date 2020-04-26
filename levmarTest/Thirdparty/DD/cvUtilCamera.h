// Seung-Chan Kim
// ver 0.1


#pragma once


#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <iostream>

#include "cvUtilDBG.h"
namespace DD {

	static void DIspCameraMatrix(cv::Mat& intrinsic_, cv::Mat& distortion_)
	{
		std::cout << "----------------------" << std::endl;
		std::cout << "Intrinsics:  " << std::endl;
		std::cout << " size= " << intrinsic_.rows << "," << intrinsic_.cols << std::endl;
		cvPrintMat(intrinsic_);
		std::cout << "----------------------" << std::endl;

		std::cout << "Distortions:  " << std::endl;
		std::cout << " size= " << distortion_.rows << "," << distortion_.cols << std::endl;
		cvPrintMat(distortion_);

		std::cout << "----------------------" << std::endl;
	}


	// based on http://dsp.stackexchange.com/a/6057
	template <typename T>
	static cv::Mat ScaleIntrinsicMat(cv::Mat& intrinsic, float scale)
	{
		cv::Mat intrinsic_scaled;
		intrinsic.copyTo(intrinsic_scaled);

		intrinsic_scaled.at<T>(0) *= scale;
		intrinsic_scaled.at<T>(2) *= scale;
		intrinsic_scaled.at<T>(4) *= scale;
		intrinsic_scaled.at<T>(5) *= scale;

		return intrinsic_scaled;

	}


	static void LoadCameraMatrix(const char* fn_cam, const char* fn_dist, cv::Mat& intrinsic_, cv::Mat& distortion_)
	{

		FileStorage fsIntrinsic(fn_cam, FileStorage::READ);
		fsIntrinsic["Intrinsics"] >> intrinsic_;
		fsIntrinsic.release();

		FileStorage fsDistortion(fn_dist, FileStorage::READ);
		fsDistortion["Distortion"] >> distortion_;
		fsDistortion.release();

	}

	template <typename T>
	static void MakeCamMat(const int w, const int h, Mat_<T>& K, Mat_<T>& D  )
	{
		T max_w_h	= MAX(h, w);	//MAX(a,b)  ((a) < (b) ? (b) : (a))
		// or float f		= std::max(w,h); // * 1.1;
		K		= (cv::Mat_<T>(3, 3) <<
					max_w_h,	0,			w/ 2.0,
					0,			max_w_h,	h / 2.0,
					0,			0,			1);


		D		= cv::Mat_<T>::zeros(1, 4);

	}


	static void NormalizeEssentialMatrix( Mat& mat )
	{
		for(int j=0; j<9; j++)
		{
			mat.at<double>(j) /= mat.at<double>(8);
		}
	}


	static void GetEssentialMat(Mat fundemental_, Mat camMat_, Mat& E_, bool bNormalize=true,  bool bVervose=false)
	{

		//Mat_<float>                 camMat;
		//intrinsic.convertTo(camMat,CV_64F);
		//cout <<camMat_.type() <<endl;
		//cout <<fundemental_.type() <<endl;

		E_ = camMat_.t() * fundemental_ * camMat_; //according to HZ (9.12)	// CHECK !!


		if(bVervose)
		{
			cout<< "Essential matrix : "<<endl;
			cvPrintMat(E_);
		}
		if(bNormalize)
		{

			NormalizeEssentialMatrix(E_);

			if(bVervose)
			{
				cout  <<endl;
				cout<< "Essential matrix (normalized) : "<<endl;
				cvPrintMat(E_);
			}

		}

	}

	template <typename T>
    static bool CheckAndFixEssentialMatrix(Mat &E, bool bVerbose=false)
	{
		//Using HZ E decomposition
		SVD svd(E,SVD::MODIFY_A);

		//check if first and second singular values are the same (as they should be)

		double singular_values_ratio = fabsf(svd.w.at<T>(0) / svd.w.at<T>(1));
		if(singular_values_ratio>1.0)
			singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]

		if (singular_values_ratio < 0.7) {
			if(bVerbose)
			{
				cout << "singular values of essential matrix are too far apart\n";
			}
			return false;
		}

		// checking and fixing
		{

			Matx33d W(0,-1,0,   //HZ 9.13
					  1,0,0,
					  0,0,1);
			Matx33d Wt(0,1,0,
					   -1,0,0,
					   0,0,1);
			Mat_<T> R1 = svd.u * Mat(W) * svd.vt; //HZ 9.19

			if(determinant(R1)+1.0 < 1e-09) {
				//according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
				cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" <<endl;
				E = -E;

				//normalizeEssentialMatrix(E);

			}

		}


		if(bVerbose)
		{
			cout<< "Essential matrix...Okay"<<endl;
		}



		return true;

	}

	static bool CheckAndFixEssentialMatrix(Mat &E, bool bVerbose=false)
	{
		//Using HZ E decomposition
		SVD svd(E,SVD::MODIFY_A);

		//check if first and second singular values are the same (as they should be)

		double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
		if(singular_values_ratio>1.0)
			singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]

		if (singular_values_ratio < 0.7) {
			if(bVerbose)
			{
				cout << "singular values of essential matrix are too far apart\n";
			}
			return false;
		}

		// checking and fixing
		{

			Matx33d W(0,-1,0,   //HZ 9.13
					  1,0,0,
					  0,0,1);
			Matx33d Wt(0,1,0,
					   -1,0,0,
					   0,0,1);
			Mat_<double> R1 = svd.u * Mat(W) * svd.vt; //HZ 9.19

			if(determinant(R1)+1.0 < 1e-09) {
				//according to http://en.wikipedia.org/wiki/Essential_matrix#Showing_that_it_is_valid
				cout << "det(R) == -1 ["<<determinant(R1)<<"]: flip E's sign" <<endl;
				E = -E;

				//normalizeEssentialMatrix(E);

			}

		}


		if(bVerbose)
		{
			cout<< "Essential matrix...Okay"<<endl;
		}



		return true;

	}








}
