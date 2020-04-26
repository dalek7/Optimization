// Seung-Chan Kim
// ver 0.1


#pragma once

#include <opencv2/opencv.hpp>
#include <fstream>
#include <string>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "cvUtilCalcHelper.h"

using namespace std;
using namespace cv;



namespace DD {
    using namespace cv;

	#ifndef draw_cross
	#define draw_cross( img, center, color, d,t )                                 \
                        line( img,	Point( center.x - d, center.y - d ),                \
                                    Point( center.x + d, center.y + d ), color, t, 0 ); \
                        line( img,	Point( center.x + d, center.y - d ),                \
                                        Point( center.x - d, center.y + d ), color, t, 0 )
	#endif

	#ifndef draw_plus
	#define draw_plus( img, center, color, d,t )                                 \
						line( img,	Point( center.x - d, center.y ),                \
									Point( center.x + d, center.y ), color, t, 0 ); \
						line( img,	Point( center.x , center.y - d ),                \
									Point( center.x , center.y + d ), color, t, 0 )
	#endif


	#ifndef draw_cross2
	#define draw_cross2( img, center, d, color, t )                                 \
						line( img,	Point( center.x - d, center.y - d ),                \
									Point( center.x + d, center.y + d ), color, t, 0 ); \
						line( img,	Point( center.x + d, center.y - d ),                \
									Point( center.x - d, center.y + d ), color, t, 0 )
	#endif

    #ifndef draw_circle
	#define draw_circle( img, center, clr, r, r2)                                   \
                        circle(img, center, r, clr);                                \
                        if(r2>0)                                                    \
                        circle(img, center, r2, clr);


	#endif

	#ifndef draw_axis2d
	#define draw_axis2d( img, center, eig0, eig1,color, d,t )                                 \
			line( img,	Point( center.val[0], center.val[1] ),                \
			Point( center.val[0] + d* eig0.val[0], center.val[1]+ d* eig0.val[1] ), color, 2, 0 ); \
			line( img,	Point( center.val[0], center.val[1] ),                \
			Point( center.val[0] + d* eig1.val[0], center.val[1]+ d* eig1.val[1] ), color, 2, 0 );
	#endif

    static void circle(Mat& src, Point2f pt, float r, Scalar clr, float r2=-1)
    {
        cv::circle(src, pt, r, clr);

        if(r2>0)
            cv::circle(src, pt, r2, clr);

    }


	static Mat PlotEpipolarLines2(const Mat fundemental, const Mat img2, vector<cv::Point2f> selPoints1)
	{

		// Right image
		// draw the left points corresponding epipolar lines in right image

		Mat imgout;
		img2.copyTo(imgout);

		std::vector<cv::Vec3f> lines1;
		cv::computeCorrespondEpilines(
			cv::Mat(selPoints1), // image points
			1,                   // in image 1 (can also be 2)
			fundemental, // F matrix
			lines1);     // vector of epipolar lines


		// for all epipolar lines
		for (typename vector<cv::Vec3f>::const_iterator it= lines1.begin();
			 it!=lines1.end(); ++it) {

				 // draw the epipolar line between first and last column
				 cv::line(imgout,cv::Point(0,-(*it)[2]/(*it)[1]),
								 cv::Point(img2.cols,-((*it)[2]+(*it)[0]*img2.cols)/(*it)[1]),
								 cv::Scalar(255,255,255));
		}


		/*
		string str0 = string_format("_K%d_vs_%d_", nKey, nImg);
		string str1 = "./out/Epipolarline_"+str0+makeFileNameWithTime("png");
		imwrite(str1, img2);
		*/

		return imgout;
	}




	const int draw_shift_bits = 4;
	const int draw_multiplier = 1 << draw_shift_bits;

	static void _prepareImgAndDrawKeypoints( const Mat& img1, const vector<KeyPoint>& keypoints1,
											 const Mat& img2, const vector<KeyPoint>& keypoints2,
											 Mat& outImg, Mat& outImg1, Mat& outImg2,
											 const Scalar& singlePointColor, int flags )
	{
		Size size( img1.cols + img2.cols, MAX(img1.rows, img2.rows) );
		if( flags & DrawMatchesFlags::DRAW_OVER_OUTIMG )
		{
			if( size.width > outImg.cols || size.height > outImg.rows )
				//CV_Error( CV_StsBadSize, "outImg has size less than need to draw img1 and img2 together" );
				cerr << "outImg has size less than need to draw img1 and img2 together" << endl;;
			outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
			outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
		}
		else
		{
			outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );
			outImg = Scalar::all(0);
			outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
			outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );

			if( img1.type() == CV_8U )
				cvtColor( img1, outImg1, CV_GRAY2BGR );
			else
				img1.copyTo( outImg1 );

			if( img2.type() == CV_8U )
				cvtColor( img2, outImg2, CV_GRAY2BGR );
			else
				img2.copyTo( outImg2 );
		}

		// draw keypoints
		if( !(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
		{
			Mat _outImg1 = outImg( Rect(0, 0, img1.cols, img1.rows) );
			drawKeypoints( _outImg1, keypoints1, _outImg1, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );

			Mat _outImg2 = outImg( Rect(img1.cols, 0, img2.cols, img2.rows) );
			drawKeypoints( _outImg2, keypoints2, _outImg2, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
		}
	}


	static void _prepareImgAndDrawKeypoints2(const Mat& img1, const vector<Point2f>& keypoints1,
		const Mat& img2, const vector<Point2f>& keypoints2,
		Mat& outImg,
		const Scalar& singlePointColor, int flags)
	{
		Size size(MAX(img1.cols, img2.cols), MAX(img1.rows, img2.rows));

		{
			outImg.create(size, CV_MAKETYPE(img1.depth(), 3));
			outImg = Scalar::all(0);


			// http://docs.opencv.org/2.4/doc/tutorials/core/adding_images/adding_images.html
			float alpha = 0.5;
			float beta = (1.0 - alpha);
			addWeighted(img1, alpha, img2, beta, 0.0, outImg);

		}

		// draw keypoints
		//if( !(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )

	}

	static void _prepareImgAndDrawKeypoints2( const Mat& img1, const vector<KeyPoint>& keypoints1,
											 const Mat& img2, const vector<KeyPoint>& keypoints2,
											 Mat& outImg,
											 const Scalar& singlePointColor, int flags )
	{
		Size size( MAX(img1.cols, img2.cols), MAX(img1.rows, img2.rows) );

		{
			outImg.create( size, CV_MAKETYPE(img1.depth(), 3) );
			outImg = Scalar::all(0);


			// http://docs.opencv.org/2.4/doc/tutorials/core/adding_images/adding_images.html
			float alpha = 0.5;
			float beta = ( 1.0 - alpha );
			addWeighted( img1, alpha, img2, beta, 0.0, outImg);

		}

		// draw keypoints
		//if( !(flags & DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS) )
		if(0)
		{

			drawKeypoints( outImg, keypoints1, outImg, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
			drawKeypoints( outImg, keypoints2, outImg, singlePointColor, flags + DrawMatchesFlags::DRAW_OVER_OUTIMG );
		}
	}

	static inline void _drawKeypoint(Mat& img, const Point2f& p, const Scalar& color, int flags)
	{
		CV_Assert(!img.empty());
		Point center(cvRound(p.x * draw_multiplier), cvRound(p.y * draw_multiplier));


		{
			// draw center with R=3
			int radius = 3 * draw_multiplier;
			circle(img, center, radius, color, 1, CV_AA, draw_shift_bits);
		}
	}

	static inline void _drawKeypoint( Mat& img, const KeyPoint& p, const Scalar& color, int flags )
	{
		CV_Assert( !img.empty() );
		Point center( cvRound(p.pt.x * draw_multiplier), cvRound(p.pt.y * draw_multiplier) );

		if( flags & DrawMatchesFlags::DRAW_RICH_KEYPOINTS )
		{
			int radius = cvRound(p.size/2 * draw_multiplier); // KeyPoint::size is a diameter

			// draw the circles around keypoints with the keypoints size
			circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );

			// draw orientation of the keypoint, if it is applicable
			if( p.angle != -1 )
			{
				float srcAngleRad = p.angle*(float)CV_PI/180.f;
				Point orient( cvRound(cos(srcAngleRad)*radius ),
							  cvRound(sin(srcAngleRad)*radius )
							);
				line( img, center, center+orient, color, 1, CV_AA, draw_shift_bits );
			}
	#if 0
			else
			{
				// draw center with R=1
				int radius = 1 * draw_multiplier;
				circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
			}
	#endif
		}
		else
		{
			// draw center with R=3
			int radius = 3 * draw_multiplier;
			circle( img, center, radius, color, 1, CV_AA, draw_shift_bits );
		}
	}
	static inline void _drawMatch( Mat& outImg, Mat& outImg1, Mat& outImg2 ,
							  const KeyPoint& kp1, const KeyPoint& kp2, const Scalar& matchColor, int flags )
	{



		RNG& rng = theRNG();
		bool isRandMatchColor = matchColor == Scalar::all(-1);
		Scalar color = isRandMatchColor ? Scalar( rng(256), rng(256), rng(256) ) : matchColor;

		_drawKeypoint( outImg1, kp1, color, flags );
		_drawKeypoint( outImg2, kp2, color, flags );

		Point2f pt1 = kp1.pt,
				pt2 = kp2.pt,
				dpt2 = Point2f( (std::min)(pt2.x+outImg1.cols, float(outImg.cols-1)), pt2.y );

		line( outImg,
			  Point(cvRound(pt1.x*draw_multiplier), cvRound(pt1.y*draw_multiplier)),
			  Point(cvRound(dpt2.x*draw_multiplier), cvRound(dpt2.y*draw_multiplier)),
			  color, 1, CV_AA, draw_shift_bits );
	}


	static inline void _drawMatch2( Mat& outImg,
		const Point2f& pt1, const Point2f& pt2, const Scalar& matchColor, int flags)
	{

		RNG& rng = theRNG();
		bool isRandMatchColor = matchColor == Scalar::all(-1);
		Scalar color = isRandMatchColor ? Scalar( rng(256), rng(256), rng(256) ) : matchColor;

		//_drawKeypoint( outImg, kp1, color, flags );
		//_drawKeypoint( outImg, kp2, color, flags );

		//Point2f dpt2 = Point2f( (std::min)(pt2.x, float(outImg.cols-1)), pt2.y );

		line( outImg,
			  Point(cvRound(pt1.x*draw_multiplier), cvRound(pt1.y*draw_multiplier)),
			  Point(cvRound(pt2.x*draw_multiplier), cvRound(pt2.y*draw_multiplier)),
			  color, 1, CV_AA, draw_shift_bits );
	}

	static inline void _drawMatch2(Mat& outImg,
		const KeyPoint& kp1, const KeyPoint& kp2, const Scalar& matchColor, int flags)
	{

		RNG& rng = theRNG();
		bool isRandMatchColor = matchColor == Scalar::all(-1);
		Scalar color = isRandMatchColor ? Scalar(rng(256), rng(256), rng(256)) : matchColor;

		_drawKeypoint(outImg, kp1, color, flags);
		_drawKeypoint(outImg, kp2, color, flags);

		Point2f pt1 = kp1.pt,
			pt2 = kp2.pt,
			dpt2 = Point2f((std::min)(pt2.x, float(outImg.cols - 1)), pt2.y);

		line(outImg,
			Point(cvRound(pt1.x*draw_multiplier), cvRound(pt1.y*draw_multiplier)),
			Point(cvRound(pt2.x*draw_multiplier), cvRound(pt2.y*draw_multiplier)),
			color, 1, CV_AA, draw_shift_bits);
	}



	static void drawMatches2( const Mat& img1, const vector<KeyPoint>& keypoints1,
					  const Mat& img2, const vector<KeyPoint>& keypoints2,
					  const vector<DMatch>& matches1to2, Mat& outImg,
					  const Scalar& matchColor, const Scalar& singlePointColor,
					  const vector<char>& matchesMask, int flags )
	{
		if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
			cout << "matchesMask must have the same size as matches1to2"<< endl;;

		Mat outImg1, outImg2;
		_prepareImgAndDrawKeypoints( img1, keypoints1, img2, keypoints2,
									 outImg, outImg1, outImg2, singlePointColor, flags );

		// draw matches
		for( size_t m = 0; m < matches1to2.size(); m++ )
		{
			if( matchesMask.empty() || matchesMask[m] )
			{
				int i1 = matches1to2[m].queryIdx;
				int i2 = matches1to2[m].trainIdx;
				CV_Assert(i1 >= 0 && i1 < static_cast<int>(keypoints1.size()));
				CV_Assert(i2 >= 0 && i2 < static_cast<int>(keypoints2.size()));

				const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
			   _drawMatch( outImg, outImg1, outImg2, kp1, kp2, matchColor, flags );
			}
		}
	}


	static void drawMatches3(const Mat& img1, const vector<Point2f>& points1,
		const Mat& img2, const vector<Point2f>& points2,
		const vector<DMatch>& matches1to2, Mat& outImg,
		const Scalar& matchColor, const Scalar& singlePointColor,
		const vector<char>& matchesMask, int flags)
	{
		if (!matchesMask.empty() && matchesMask.size() != matches1to2.size())
			cerr<< "matchesMask must have the same size as matches1to2" << endl;

		Mat outImg1;// , outImg2;
		_prepareImgAndDrawKeypoints2(img1, points1, img2, points2,
			outImg1, singlePointColor, flags);

		Mat outImg1c;
		if(outImg1.channels()==1)
			cvtColor(outImg1, outImg1c, COLOR_GRAY2BGR);
		else
			outImg1.copyTo(outImg1c);

		//if(outImg1

		// draw

		for (size_t m = 0; m < matches1to2.size(); m++)
		{
			if (matchesMask.empty() || matchesMask[m])
			{
				int i1 = matches1to2[m].queryIdx;
				int i2 = matches1to2[m].trainIdx;
				//CV_Assert(i1 >= 0 && i1 < static_cast<int>(points1.size()));
				//CV_Assert(i2 >= 0 && i2 < static_cast<int>(points2.size()));

				const Point2f kp1 = points1[i1];
				const Point2f kp2 = points2[i2];
				_drawMatch2(outImg1c, kp1, kp2, matchColor, flags);
			}
		}

		outImg1c.copyTo(outImg);
	}


	static void drawMatches3( const Mat& img1, const vector<KeyPoint>& keypoints1,
					  const Mat& img2, const vector<KeyPoint>& keypoints2,
					  const vector<DMatch>& matches1to2, Mat& outImg,
					  const Scalar& matchColor, const Scalar& singlePointColor,
					  const vector<char>& matchesMask, int flags )
	{
		if( !matchesMask.empty() && matchesMask.size() != matches1to2.size() )
			cerr << "matchesMask must have the same size as matches1to2" << endl;

		Mat outImg1;// , outImg2;
		_prepareImgAndDrawKeypoints2(	img1, keypoints1, img2, keypoints2,
										outImg1, singlePointColor, flags);


		// draw
		if (0);
		for( size_t m = 0; m < matches1to2.size(); m++ )
		{
			if( matchesMask.empty() || matchesMask[m] )
			{
				int i1 = matches1to2[m].queryIdx;
				int i2 = matches1to2[m].trainIdx;
				CV_Assert(i1 >= 0 && i1 < static_cast<int>(keypoints1.size()));
				CV_Assert(i2 >= 0 && i2 < static_cast<int>(keypoints2.size()));

				const KeyPoint &kp1 = keypoints1[i1], &kp2 = keypoints2[i2];
				_drawMatch2(outImg1, kp1, kp2, matchColor, flags);
			}
		}

		outImg1.copyTo(outImg);
	}


	static Mat DrawMatches(Mat img_1, vector<Point2f> &points1, Mat img_2, vector<Point2f> &points2, vector< DMatch > matches, Scalar clr = Scalar::all(-1), const string title = "")
	{
		Mat img_matches;
		drawMatches3(img_1, points1, img_2, points2,
			matches, img_matches, clr, clr,
			vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);


		if (0)
		{
			namedWindow(title, WINDOW_NORMAL);
			imshow(title, img_matches);

			int h1 = GetHeightFromWithWidth(img_matches, 640 * 2);
			resizeWindow(title, 640 * 2, h1);
		}
		return img_matches;
	}



	static Mat DrawMatches(Mat img_1, vector<KeyPoint> &keypoints1, Mat img_2, vector<KeyPoint> &keypoints2, vector< DMatch > matches, Scalar clr = Scalar::all(-1),  const string title = "")
	{
		Mat img_matches;
		drawMatches3( img_1, keypoints1, img_2, keypoints2,
						matches, img_matches, clr, clr,
						vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


		if(0)
		{
			namedWindow(title, WINDOW_NORMAL );
			imshow( title, img_matches );

			int h1 = GetHeightFromWithWidth(img_matches, 640*2);
			resizeWindow(title, 640*2,h1);
		}
		return img_matches;
	}

    static void DrawSinglePoint(cv::Mat image1, Point2f pt, cv::Scalar clr = cv::Scalar(255,255,255), int pointtype = 'o', float sz = 1, float thickness = 1)
    {
        if(pointtype==0 || pointtype == 'o' || pointtype == 'O')
				cv::circle(image1,pt,sz,clr,thickness);

        else if(pointtype==1 || pointtype == 'x' || pointtype == 'X')
        {
            float d = sz;
            float t = thickness;

            cv::line( image1,	cv::Point( pt.x - d, pt.y - d ),
                                cv::Point( pt.x + d, pt.y + d ), clr, t, 0 );
            cv::line( image1,	cv::Point( pt.x + d, pt.y - d ),
                                cv::Point( pt.x - d, pt.y + d ), clr, t, 0 );

        }
        else if(pointtype==2 || pointtype == 43)   //'+' 20160801
        {
            float d = sz;
            float t = thickness;

            cv::line( image1,	cv::Point( pt.x - d, pt.y ),
                                cv::Point( pt.x + d, pt.y ), clr, t, 0 );
            cv::line( image1,	cv::Point( pt.x , pt.y - d ),
                                cv::Point( pt.x , pt.y + d ), clr, t, 0 );

        }

    }

	template<typename T>
	static void DrawPointsOnTheImage(vector<T> selPoints1, Mat& image1, bool addid=true, cv::Scalar clr = cv::Scalar(255,255,255), int pointtype = 'o', float sz = 1, float thickness = 1 )
	{
		// check by drawing the points
		char txt[8];
		//Scalar color = Scalar(0,0,0);// 255
		typename vector<T>::const_iterator it= selPoints1.begin();
		int i = 0;
		while (it!=selPoints1.end()) {

			// draw a circle at each corner location

			if(pointtype==0 || pointtype == 'o' || pointtype == 'O')
				cv::circle(image1,*it,sz,clr,thickness);
			else if(pointtype==1 || pointtype == 'x' || pointtype == 'X')
			{
				float d = sz;
				float t = thickness;

				cv::line( image1,	cv::Point( it->x - d, it->y - d ),
									cv::Point( it->x + d, it->y + d ), clr, t, 0 );
				cv::line( image1,	cv::Point( it->x + d, it->y - d ),
									cv::Point( it->x - d, it->y + d ), clr, t, 0 );

			}
			else if(pointtype==2 || pointtype == 43)   //'+' 20160801
			{
                float d = sz;
				float t = thickness;

                cv::line( image1,	cv::Point( it->x - d, it->y ),
									cv::Point( it->x + d, it->y ), clr, t, 0 );
				cv::line( image1,	cv::Point( it->x , it->y - d ),
									cv::Point( it->x , it->y + d ), clr, t, 0 );

			}

			if(addid)
			{
				sprintf(txt, "%d", i++);
				putText(image1, txt, Point(it->x+10,it->y), FONT_HERSHEY_SIMPLEX, 0.5, clr,1);
			}
			++it;
		}
	}


	template<typename T>
	static cv::Mat DrawConnectionBetweenPointsWithMatch(const cv::Mat img0, const vector<T> points0, const vector<T> points1, const vector<DMatch> matches, cv::Scalar clr = cv::Scalar(255, 255, 255))
	{

		if (points0.size() != points1.size())
		{
			return Mat();
		}

		cv::Mat img_matches;
		img0.copyTo(img_matches);

		for (typename vector<DMatch>::const_iterator it = matches.begin(); it != matches.end(); ++it)

		//for (int i = 0; i<points0.size(); i++)
		{

			T p0 = points0[it->queryIdx];
			T p1 = points1[it->trainIdx];

			cv::line(img_matches, p0, p1, clr);
		}


		DrawPointsOnTheImage<T>(points0, img_matches, false);
		DrawPointsOnTheImage<T>(points1, img_matches, false);



		return img_matches;
	}


	template<typename T>
	static cv::Mat DrawConnectionBetweenPoints(const cv::Mat img0, const vector<T> points0, const vector<T> points1, cv::Scalar clr = cv::Scalar(255,255,255))
	{

		if(points0.size() != points1.size())
		{
			return Mat();
		}

		cv::Mat img_matches;
		img0.copyTo(img_matches);


		for(int i=0; i<points0.size(); i++)
		{
			T p0 = points0[i];
			T p1 = points1[i];

			cv::line(img_matches, p0, p1, clr);
		}


		DrawPointsOnTheImage<T>(points0, img_matches, false);
		DrawPointsOnTheImage<T>(points1, img_matches, false);



		return img_matches;
	}




	static void PlotEpipolarLines(Mat fundemental, Mat img2, vector<cv::Point2f> selPoints1)
	{

		// Right image
		// draw the left points corresponding epipolar lines in right image
		std::vector<cv::Vec3f> lines1;
		cv::computeCorrespondEpilines(
			cv::Mat(selPoints1), // image points
			1,                   // in image 1 (can also be 2)
			fundemental, // F matrix
			lines1);     // vector of epipolar lines


		// for all epipolar lines
		for (vector<cv::Vec3f>::const_iterator it= lines1.begin();
			 it!=lines1.end(); ++it) {

			//if (fabs((*it)[1]) < FLT_EPSILON) continue;
			if (fabs((*it)[1]) < 0.05) continue;	// TO AVOID UGLY LINES
				 // draw the epipolar line between first and last column
				 cv::line(img2,cv::Point(0,-(*it)[2]/(*it)[1]),
								 cv::Point(img2.cols,-((*it)[2]+(*it)[0]*img2.cols)/(*it)[1]),
								 cv::Scalar(255,255,255));
		}




		//string str0 = string_format("_K%d_vs_%d_", nKey, nImg);
		//string str1 = "./out/Epipolarline_"+str0+makeFileNameWithTime("png");
		//imwrite(str1, img2);
	}


	/*
	static void DrawPointsOnTheImage(vector<cv::Point2f> selPoints1, Mat& image1, Scalar clr = Scalar(255,255,2555), bool addid=false)
	{
		// check by drawing the points
		char txt[8];
		Scalar color = Scalar(0, 0, 0);// 255
		vector<cv::Point2f>::const_iterator it = selPoints1.begin();
		int i = 0;
		while (it != selPoints1.end()) {

			// draw a circle at each corner location
			cv::circle(image1, *it, 3, clr, 2);

			if (addid)
			{
				sprintf(txt, "%d", i++);
				putText(image1, txt, Point(it->x + 10, it->y), FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
			}
			++it;
		}


	}
	*/
}
