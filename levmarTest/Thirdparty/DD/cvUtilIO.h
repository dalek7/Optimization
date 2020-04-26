// Seung-Chan Kim
// ver 0.1

#ifndef __CVUTILIO_H__
#define __CVUTILIO_H__



#include "opencv2/opencv.hpp"
#include <fstream>
#include <string>
#include <iostream>

using namespace std;
using namespace cv;


#include "CSVRow.h"
#include "DDUtil.h"



#include "cvCustomFormat.h"



namespace DD {

    // to prevent strange cv::waitKey() behavior in linux.
    // example : returning 0 when unfocused.
    static char waitKey2()
    {
        char ch=0;
        while(!((ch & 0xEFFFFF) != 0))
        {
            ch = cv::waitKey();
        }
        return ch;
    }

    static void waitSpecificKey(char key_allow=27) // esc
    {
        while((cv::waitKey() & 0xEFFFFF) != key_allow);
    }

	static std::vector<cv::Point3d> CloudPointsToPoints(const std::vector<CloudPoint> cpts) {
		std::vector<cv::Point3d> out;
		for (unsigned int i = 0; i<cpts.size(); i++) {
			out.push_back(cpts[i].pt);
		}
		return out;
	}


	template <typename T>
	static std::vector<T> CloudPointsToPointsT(const std::vector<CloudPoint> cpts) {
		std::vector<T> out;
		for (unsigned int i = 0; i<cpts.size(); i++) {
			T pt1 = T(cpts[i].pt.x, cpts[i].pt.y, cpts[i].pt.z);

			out.push_back(pt1);
		}
		return out;
	}



	template <typename T>
	static cv::Mat LoadXx4Mat(string fn, int x=4, const char* tok = ",")
	{
		string line;
		ifstream myfile(fn);

		Mat P(x, 4, CV_32F);

		if (myfile.is_open())
		{
			int nrow = 0;
			while (getline(myfile, line))
			{

				vector<string> tokens;
				tokenize(line, tokens, tok);

				float c0 = atof(tokens.at(0).c_str());
				float c1 = atof(tokens.at(1).c_str());
				float c2 = atof(tokens.at(2).c_str());
				float c3 = atof(tokens.at(3).c_str());

				//x1.push_back(Point3f(x, y, z));

				Mat row1 = (cv::Mat_<T>(1, 4) <<
					c0, c1, c2, c3);


				row1.copyTo(P.row(nrow++));

			}
			myfile.close();
		}


		return P;
	}

	template <typename T>
	static cv::Mat LoadMxNMat(string fn, const char* tok = ",", int n_row = -1, int n_col = -1)
	{

		if(n_row == -1 || n_col == -1)
		{

			string line;
			ifstream myfile(fn);
			if (!myfile.is_open())
				return Mat();


			int nrow = 0;
			while (getline(myfile, line))
			{
				vector<string> tokens;
				DD::tokenize(line, tokens, tok);

				if(nrow==0)				n_col	= tokens.size();
				nrow++;

			}


			n_row	= nrow;


			//n_col = tokens.size();
		}

		cout << "Loading... " << n_row << " x " << n_col << " mat" << endl;

		string line;
		ifstream myfile(fn);
		Mat P = Mat_<T>(n_row, n_col);
		if (myfile.is_open())
		{
			int nrow = 0;
			while (getline(myfile, line))
			{

				//cout << " >> "<< line << '\n';
				vector<string> tokens;
				DD::tokenize(line, tokens, tok);

				for(int i=0;i<tokens.size(); i++)
				{
                    if(tokens.at(i) != "\r" )   //20160801
					P.at<T>(n_row, i) = atof(tokens.at(i).c_str());

				}

				nrow++;

			}

		}
		else
		{
            //myfile.close();
			return Mat();

        }

        myfile.close();
		return P.clone();

	}

	template <typename T>
	static cv::Mat Load3x4Mat(string fn, const char* tok = ",")
	{
		string line;
		ifstream myfile(fn);

		//Mat P(3, 4, CV_32F);
		Mat P = Mat_<T>(3, 4);
		if (myfile.is_open())
		{
			int nrow = 0;
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				DD::tokenize(line, tokens, tok);

				float c0 = atof(tokens.at(0).c_str());
				float c1 = atof(tokens.at(1).c_str());
				float c2 = atof(tokens.at(2).c_str());
				float c3 = atof(tokens.at(3).c_str());

				//x1.push_back(Point3f(x, y, z));

				Mat row1 = (cv::Mat_<T>(1, 4) <<
					c0, c1, c2, c3);


				row1.copyTo(P.row(nrow++));

			}
			myfile.close();
		}
		else
			return Mat();


		return P.clone();
	}

    template <typename T>
	static cv::Mat Load3x3Mat(string fn, const char* tok = ",")
	{
		string line;
		ifstream myfile(fn);

		//Mat P(3, 4, CV_32F);
		Mat P = Mat_<T>(3, 3);
		if (myfile.is_open())
		{
			int nrow = 0;
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				DD::tokenize(line, tokens, tok);

				float c0 = atof(tokens.at(0).c_str());
				float c1 = atof(tokens.at(1).c_str());
				float c2 = atof(tokens.at(2).c_str());


				//x1.push_back(Point3f(x, y, z));

				Mat row1 = (cv::Mat_<T>(1, 3) <<
					c0, c1, c2);


				row1.copyTo(P.row(nrow++));

			}
			myfile.close();
		}
		else
			return Mat();


		return P.clone();
	}



	static vector<Point2f> LoadData2f(string fn, const char* delimiter = ",")
	{

		vector<Point2f> x1;
		string line;
		ifstream myfile(fn);

		if (myfile.is_open())
		{
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				tokenize(line, tokens, delimiter);

				float x = atof(tokens.at(0).c_str());
				float y = atof(tokens.at(1).c_str());
				//float z = atof(tokens.at(2).c_str());

				x1.push_back(Point2f(x, y));

				//cout <<" (" << tokens.at(0) << ", " <<tokens.at(1) << ", " << tokens.at(2) << ")" <<endl ;
				//cout <<" (" << x << ", " << y << ", " << z << ")" <<endl ;

			}
			myfile.close();
		}

		return x1;

	}


	template<typename T>
	static vector<T> LoadData2dim(string fn, const char* delimiter = ",")
	{

		vector<T> x1;
		string line;
		ifstream myfile(fn);

		if (myfile.is_open())
		{
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				DD::tokenize(line, tokens, delimiter);

				float x = atof(tokens.at(0).c_str());
				float y = atof(tokens.at(1).c_str());
				//float z = atof(tokens.at(2).c_str());

				x1.push_back(T(x, y));

				//cout <<" (" << tokens.at(0) << ", " <<tokens.at(1) << ", " << tokens.at(2) << ")" <<endl ;
				//cout <<" (" << x << ", " << y << ", " << z << ")" <<endl ;

			}
			myfile.close();
		}

		return x1;

	}



	static vector<Point3d> LoadData3d(string fn, const char* tok = ",")
	{
		vector<Point3d> x1;
		string line;
		ifstream myfile(fn);

		if (myfile.is_open())
		{
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				tokenize(line, tokens, tok);

				float x = atof(tokens.at(0).c_str());
				float y = atof(tokens.at(1).c_str());
				float z = atof(tokens.at(2).c_str());

				x1.push_back(Point3d(x, y, z));
			}
			myfile.close();
		}

		return x1;
	}

	static vector<Point3f> LoadData3f(string fn, const char* tok = ",")
	{
		vector<Point3f> x1;
		string line;
		ifstream myfile(fn);

		if (myfile.is_open())
		{
			while (getline(myfile, line))
			{

				//cout << line << '\n';
				vector<string> tokens;
				tokenize(line, tokens, tok);

				float x = atof(tokens.at(0).c_str());
				float y = atof(tokens.at(1).c_str());
				float z = atof(tokens.at(2).c_str());

				x1.push_back(Point3f(x, y, z));
			}
			myfile.close();
		}

		return x1;
	}


	static void cvFPrintMat(const CvMat& mat, const char* fn)
	{
		FILE *fp = fopen(fn, "w+");
		int i, j;
		for (i = 0; i < mat.rows; i++)
		{
			for (j = 0; j < mat.cols; j++)
			{
				fprintf(fp, "%f ", cvmGet(&mat, i, j));
			}
			fprintf(fp, "\n");
		}

		fclose(fp);
	}

	static bool writePoints2(const vector<Point2f>& points, string filename)
	{

		vector<Point2f>::const_iterator it = points.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for (; it != points.end(); ++it)
		{
			fprintf(fp, "%f\t%f\n", it->x, it->y);

		}
		fclose(fp);

		return 1;
	}




	static bool writePoints2(const vector<Point3f>& points, string filename)
	{

		vector<Point3f>::const_iterator it = points.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for (; it != points.end(); ++it)
		{
			fprintf(fp, "%f\t%f\t%f\n", it->x, it->y, it->z);

		}
		fclose(fp);

		return 1;
	}


	static bool writePoints2(const vector<Point3d>& points, string filename)
	{

		vector<Point3d>::const_iterator it = points.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for (; it != points.end(); ++it)
		{
			fprintf(fp, "%f\t%f\t%f\n", it->x, it->y, it->z);

		}
		fclose(fp);

		return 1;
	}


	template <typename T>
	static bool writePoints2T(const vector<Point3_<T> >& points, string filename)
	{

		typename vector<Point3_<T> >::const_iterator it = points.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for (; it != points.end(); ++it)
		{
			fprintf(fp, "%f\t%f\t%f\n", it->x, it->y, it->z);

		}
		fclose(fp);

		return 1;
	}



	static void cvFPrintMat(const CvMat& mat, const char* fn);
	// output from cv::triangulatePoints()
	static bool writePoints2(const Mat& points4D, string filename)
	{

		//cvFPrintMat(points4D, filename.c_str());

		Mat mat = points4D.t();

		FILE *fp = fopen(filename.c_str(), "w+");
		int i;
		for (i = 0; i < mat.rows; i++)
		{

			double x = mat.at<float>(i, 0) / mat.at<float>(i, 3);
			double y = mat.at<float>(i, 1) / mat.at<float>(i, 3);
			double z = mat.at<float>(i, 2) / mat.at<float>(i, 3);
			fprintf(fp, "%f\t%f\t%f\n", x, y, z);
		}

		fclose(fp);
		return 1;
	}


	static bool writePoints(const vector<Point3f>& points, string filename)
	{
		std::ofstream out(filename);

		//out << features;
		out << points;
		//int sz = features.size();
		//out << "Size: "<< sz;

		out.close();

		return 1;
	}


	static bool writePoints(const vector<Point2f>& points, string filename)
	{
		std::ofstream out(filename);

		//out << features;
		out << points;
		//int sz = features.size();
		//out << "Size: "<< sz;

		out.close();

		return 1;
	}



	// Matrix ver.
	template <typename T>
	static bool writePoints2m(const Mat& clouds3d, string filename)
	{
		vector<T> points;
		clouds3d.copyTo(points);


		typename vector<T>::const_iterator it = points.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for(; it!= points.end(); ++it)
		{
			fprintf(fp, "%f\t%f\t%f\n", it->x, it->y, it->z);

		}
		fclose(fp);

		return 1;
	}

	template<typename T>
	void SnapshotCurrentCloud(std::vector<T> pcloud, std::string fn)
	{
		int numpts3D = pcloud.size();
		std::vector<cv::Point3f> points;

		points.resize(numpts3D);
		for(int i=0; i<numpts3D; i++) {
			points[i].x = pcloud[i].pt.x;
			points[i].y = pcloud[i].pt.y;
			points[i].z = pcloud[i].pt.z;
		}


		writePoints2(points, fn);
		//writeRTs2(R, T, "outMay/motBefore.txt");
	}


	static bool writeMatrix3x4dVector(const vector<Matx34d>& vec, string filename)
	{

		vector<Matx34d>::const_iterator it = vec.begin();
		FILE *fp = fopen(filename.c_str(), "w+");

		for(; it!= vec.end(); ++it)
		{
			Matx34d p = *it;




			fprintf(fp, "%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\n", p.val[0], p.val[1],p.val[2],p.val[3],p.val[4],p.val[5],p.val[6],p.val[7],p.val[8],p.val[9],p.val[10],p.val[11] );

		}
		fclose(fp);

		return 1;
	}


	static void SaveGoodKeypoints(const vector< DMatch > good_matches, const vector<KeyPoint> keypoints1,const  vector<KeyPoint> keypoints2, const string filename)
	{
		vector<int> pointIndexes1;
		vector<int> pointIndexes2;

		vector<Point2f> points1o;
		vector<Point2f> points2o;

		for (vector<cv::DMatch>::const_iterator it= good_matches.begin(); it!= good_matches.end(); ++it) {

				 // Get the indexes of the selected matched keypoints
				 pointIndexes1.push_back(it->queryIdx);
				 pointIndexes2.push_back(it->trainIdx);
		}


		for(unsigned int i = 0; i < pointIndexes1.size(); i++ )
		{
			int idx = pointIndexes1[i];
			if( idx >= 0 )
				points1o.push_back(keypoints1[idx].pt);

		}

		for(unsigned int i = 0; i < pointIndexes2.size(); i++ )
		{
			int idx = pointIndexes2[i];
			if( idx >= 0 )
				points2o.push_back(keypoints2[idx].pt);

		}

		// SAVE
		//writePoints2(ps1, fn1);

		vector<Point2f>::const_iterator it	= points1o.begin();
		vector<Point2f>::const_iterator it2 = points2o.begin();
		FILE *fp = fopen(filename.c_str(), "w+");
		for(; it!= points1o.end(); ++it, ++it2)
		{
			fprintf(fp, "%f\t%f\t%f\t%f\n", it->x, it->y, it2->x, it2->y);

		}
		fclose(fp);


	}


}

#endif
