#pragma once

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cminpack.h>
#define real __cminpack_real__

using namespace cv;

// 観測データを定義する構造体
typedef struct {
	int m;
	real *y;
	Mat_<double>* K;
	Mat_<double>* P1;
	Mat_<double>* P2;
	Point3d* pt3d;
	Point2f* pt1;
	Point2f* pt2;

} fcndata_t;

class Reconstruction {
public:
	Reconstruction();

	Mat findFundamentalMat(std::vector<Point2f>& pts1, std::vector<Point2f>& pts2, std::vector<uchar>& status);
	Mat_<double> computeFundamentalMatByEightPoints(std::vector<Point2f>& pts1, Mat_<double>& T1, std::vector<Point2f>& pts2, Mat_<double>& T2);
	Mat_<double> normalizePoints(std::vector<Point2f>& pts, std::vector<Point2f>& normalized_pts);
	void writeEpipolarLines(char* filename, Mat& img, std::vector<Point2f>& pts1, Mat& F, std::vector<Point2f>& pts2, int whichImage);

	Mat computeEpipole(Mat& F, int whichImage);
	void computeProjectionMat(Matx33d E, Mat_<double>& R1, Mat_<double>& T1, Mat_<double>& R2, Mat_<double>& T2);

	double unprojectPoints(const Mat_<double>& K, const Mat_<double>& R1, const Mat_<double>& T1, const Mat_<double>& R2, const Mat_<double>& T2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, Mat_<double>& P1, Mat_<double>& P2);
	bool unprojectPoints(const Mat_<double>& K, const Mat_<double>& Kinv, const Mat_<double>& P, const Mat_<double>& P1, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d, double& error);
	Mat_<double> triangulate(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1);
	Mat_<double> iterativeTriangulation(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1);

	void sampson(Mat_<double>& F, std::vector<Point2f>& pts1, std::vector<Point2f>& pts2);
	bool decomposeEtoRandT(const Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2);

	void bundleAdjustment(Mat_<double>& F, Mat_<double>& P1, Mat_<double>& P2, Mat_<double>& K, Point2f& pt1, Point2f& pt2, Point3d& pt3d);
	static int fcn(void *p, int m, int n, const real *x, real *fvec, int iflag);
};

