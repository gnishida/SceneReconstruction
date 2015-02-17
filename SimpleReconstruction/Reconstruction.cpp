#include "Reconstruction.h"

using namespace cv;

Reconstruction::Reconstruction() {
}

Mat Reconstruction::findFundamentalMat(std::vector<cv::Point2f>& pts1, std::vector<cv::Point2f>& pts2, std::vector<uchar>& status) {
	//return cv::findFundamentalMat(pts1, pts2, CV_FM_RANSAC, 3, 0.99, status);

	double best_confidence = 0.0;
	cv::Mat best_F;

	// 全ての点をnormalizeする
	std::vector<Point2f> normalized_pts[2];
	Mat_<double> T1 = normalizePoints(pts1, normalized_pts[0]);
	Mat_<double> T2 = normalizePoints(pts2, normalized_pts[1]);

	Mat F = computeFundamentalMatByEightPoints(normalized_pts[0], T1, normalized_pts[1], T2);

	return F;
}

Mat_<double> Reconstruction::computeFundamentalMatByEightPoints(std::vector<Point2f>& pts1, Mat_<double>& T1, std::vector<Point2f>& pts2, Mat_<double>& T2) {
	Mat_<double> A(pts1.size(), 9);

	for (int i = 0; i < pts1.size(); ++i) {
		A(i, 0) = pts1[i].x * pts2[i].x;
		A(i, 1) = pts1[i].y * pts2[i].x;
		A(i, 2) = pts2[i].x;
		A(i, 3) = pts1[i].x * pts2[i].y;
		A(i, 4) = pts1[i].y * pts2[i].y;
		A(i, 5) = pts2[i].y;
		A(i, 6) = pts1[i].x;
		A(i, 7) = pts1[i].y;
		A(i, 8) = 1;
	}

	SVD svd;
	Mat_<double> u, w, vt;
	svd.compute(A, u, w, vt);
	Mat_<double> F_hat = (Mat_<double>(3, 3) << vt(vt.rows-1, 0), vt(vt.rows-1, 1), vt(vt.rows-1, 2),
											vt(vt.rows-1, 3), vt(vt.rows-1, 4), vt(vt.rows-1, 5),
											vt(vt.rows-1, 6), vt(vt.rows-1, 7), vt(vt.rows-1, 8));

	// Fを、無理やりrank 2にする
	SVD svd2(F_hat);
	Mat_<double> w_prime = (Mat_<double>(3, 3) << svd2.w.at<double>(0, 0), 0, 0,
													0, svd2.w.at<double>(1, 0), 0,
													0, 0, 0);
	Mat_<double> F_hat_prime = svd2.u * w_prime * svd2.vt;

	// normalizeする前に戻す
	Mat_<double> F = T2.t() * F_hat_prime * T1;

	return F;
}

Mat_<double> Reconstruction::normalizePoints(std::vector<Point2f>& pts, std::vector<Point2f>& normalized_pts) {
	// centroidを計算する
	double total_x = 0;
	double total_y = 0;
	for (int i = 0; i < pts.size(); ++i) {
		total_x += pts[i].x;
		total_y += pts[i].y;
	}

	Point2f centroid(total_x / pts.size(), total_y / pts.size());

	// average distanceを計算する
	double total_distance = 0;
	for (int i = 0; i < pts.size(); ++i) {
		total_distance += norm(pts[i] - centroid);
	}
	double avg_distance = total_distance / pts.size();

	double scale = sqrt(2.0) / avg_distance;
	Mat_<double> T = (Mat_<double>(3, 3) << scale, 0, -centroid.x * scale,
											0, scale, -centroid.y * scale,
											0, 0, 1);

	// normalizeする
	normalized_pts.resize(pts.size());
	for (int i = 0; i < pts.size(); ++i) {
		Mat_<double> p = (Mat_<double>(3, 1) << pts[i].x, pts[i].y, 1);
		Mat_<double> normalized_p = T * p;

		normalized_pts[i] = Point2f(normalized_p(0, 0), normalized_p(1, 0));
	}

	return T;
}

void Reconstruction::writeEpipolarLines(char* filename, Mat& img, std::vector<Point2f>& pts1, Mat& F, std::vector<Point2f>& pts2, int whichImage) {
	Mat tempImg;
	flip(img, tempImg, 0);

	std::vector<Vec3f> lines;
	computeCorrespondEpilines(pts2, whichImage, F, lines);

	for (int i = 0; i < lines.size(); ++i) {
		int x1 = 0;
		int y1 = -lines[i][2] / lines[i][1];
		int x2 = tempImg.cols - 1;
		int y2 = -(lines[i][0] + lines[i][2]) / lines[i][1];

		Scalar color(i * 10 % 255, i * 20 % 255, i * 40 % 255);
		cv::line(tempImg, Point(x1, y1), Point(x2, y2), color, 2);
	}
	for (int i = 0; i < pts1.size(); ++i) {
		Scalar color(i * 10 % 255, i * 20 % 255, i * 40 % 255);
		cv::circle(tempImg, pts1[i], 5, color, 3);
	}

	flip(tempImg, tempImg, 0);
	imwrite(filename, tempImg);
}

Mat Reconstruction::computeEpipole(cv::Mat& F, int whichImage) {
	cv::Mat e(3, 1, CV_64F);
	if (whichImage == 1) {
		cv::Mat u, d, v;
		cv::SVD::compute(F, u, d, v);
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	} else {
		cv::Mat u, d, v;
		cv::SVD::compute(F.t(), u, d, v);
		e.at<double>(0, 0) = v.at<double>(v.rows - 1, 0);
		e.at<double>(1, 0) = v.at<double>(v.rows - 1, 1);
		e.at<double>(2, 0) = v.at<double>(v.rows - 1, 2);
	}

	return e;
}

void Reconstruction::computeProjectionMat(Matx33d E, Mat_<double>& R1, Mat_<double>& T1, Mat_<double>& R2, Mat_<double>& T2) {
	SVD svd(E);
	Matx33d W(0, -1, 0, 1, 0, 0, 0, 0, 1);
	Matx33d Winv(0, 1, 0, -1, 0, 0, 0, 0, 1);
	R1 = svd.u * Mat(W) * svd.vt;
	R2 = svd.u * Mat(W.t()) * svd.vt;
	T1 = svd.u.col(2);
	T2 = -svd.u.col(2);
}

double Reconstruction::unprojectPoints(const Mat_<double>& P1, const Mat_<double>& P2, const std::vector<cv::Point2f>& pts1, const std::vector<cv::Point2f>& pts2, std::vector<cv::Point3d>& pts3d) {
	pts3d.clear();
	std::vector<double> errors;

	int numFront = 0;

	for (int i = 0; i < pts1.size(); ++i) {
		Point3d u1(pts1[i].x, pts1[i].y, 1.0);
		
		Point3d u2(pts2[i].x, pts2[i].y, 1.0);

		Mat_<double> X = iterativeTriangulation(u1, P1, u2, P2);

		std::cout << "X:\n" << X << std::endl;
		cv::Point3d p = cv::Point3d(X(0), X(1), X(2));
		pts3d.push_back(p);
		
		// reprojection errorを計算する
		cv::Mat_<double> pt1_3d_hat = Mat_<double>(P1) * Mat_<double>(X);
		Point2f pt1_hat(pt1_3d_hat(0, 0) / pt1_3d_hat(2, 0), pt1_3d_hat(1, 0) / pt1_3d_hat(2, 0));
		std::cout << "projected point1: " << pt1_hat << " (observed: " << pts1[i] << ") E=" << norm(pt1_hat - pts1[i]) << std::endl;
		errors.push_back(norm(pt1_hat - pts1[i]));

		cv::Mat_<double> pt2_3d_hat = Mat_<double>(P2) * Mat_<double>(X);
		Point2f pt2_hat(pt2_3d_hat(0, 0) / pt2_3d_hat(2, 0), pt2_3d_hat(1, 0) / pt2_3d_hat(2, 0));
		std::cout << "projected point2: " << pt2_hat << " (observed: " << pts2[i] << ") E=" << norm(pt2_hat - pts1[i]) << std::endl;
		errors.push_back(norm(pt2_hat - pts1[i]));

	}

	return mean(errors)[0];
}

Mat_<double> Reconstruction::triangulate(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1) {
    Mat_<double> A = (Mat_<double>(4, 3) << u.x*P(2,0)-P(0,0), u.x*P(2,1)-P(0,1), u.x*P(2,2)-P(0,2),
											u.y*P(2,0)-P(1,0), u.y*P(2,1)-P(1,1), u.y*P(2,2)-P(1,2),
											u1.x*P1(2,0)-P1(0,0), u1.x*P1(2,1)-P1(0,1), u1.x*P1(2,2)-P1(0,2),
											u1.y*P1(2,0)-P1(1,0), u1.y*P1(2,1)-P1(1,1), u1.y*P1(2,2)-P1(1,2));
    Mat_<double> B = (Mat_<double>(4, 1) << -(u.x*P(2,3) - P(0,3)),
											-(u.y*P(2,3) - P(1,3)),
											-(u1.x*P1(2,3) - P1(0,3)),
											-(u1.y*P1(2,3) - P1(1,3)));
 
    Mat_<double> X;
    solve(A,B,X,DECOMP_SVD);

    return X;
}

Mat_<double> Reconstruction::iterativeTriangulation(const Point3d& u, const Mat_<double>& P, const Point3d& u1, const Mat_<double>& P1) {
	double wi = 1, wi1 = 1;
	Mat_<double> X(4,1);
    for (int i=0; i<10; i++) { //Hartley suggests 10 iterations at most
        Mat_<double> X_ = triangulate(u,P,u1,P1);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
         
        //recalculate weights
        double p2x = Mat_<double>(P.row(2)*X)(0);
        double p2x1 = Mat_<double>(P1.row(2)*X)(0);
         
        //breaking point
        if(fabsf(wi - p2x) <= 1e-7 && fabsf(wi1 - p2x1) <= 1e-7) break;
         
        wi = p2x;
        wi1 = p2x1;
         
        //reweight equations and solve
        Mat_<double> A = (Mat_<double>(4, 3) << (u.x*P(2,0)-P(0,0))/wi, (u.x*P(2,1)-P(0,1))/wi, (u.x*P(2,2)-P(0,2))/wi,
												(u.y*P(2,0)-P(1,0))/wi, (u.y*P(2,1)-P(1,1))/wi, (u.y*P(2,2)-P(1,2))/wi,
												(u1.x*P1(2,0)-P1(0,0))/wi1, (u1.x*P1(2,1)-P1(0,1))/wi1, (u1.x*P1(2,2)-P1(0,2))/wi1,
												(u1.y*P1(2,0)-P1(1,0))/wi1, (u1.y*P1(2,1)-P1(1,1))/wi1, (u1.y*P1(2,2)-P1(1,2))/wi1);
        Mat_<double> B = (Mat_<double>(4,1) <<    -(u.x*P(2,3)    -P(0,3))/wi,
                          -(u.y*P(2,3)  -P(1,3))/wi,
                          -(u1.x*P1(2,3)    -P1(0,3))/wi1,
                          -(u1.y*P1(2,3)    -P1(1,3))/wi1
                          );
         
        solve(A,B,X_,DECOMP_SVD);
        X(0) = X_(0); X(1) = X_(1); X(2) = X_(2); X(3) = 1.0;
    }
    return X;
}

void Reconstruction::sampson(Mat_<double>& F, std::vector<Point2f>& pts1, std::vector<Point2f>& pts2) {
	for (int i = 0; i < pts1.size(); ++i) {
		Mat_<double> x1 = (Mat_<double>(3, 1) << pts1[i].x, pts1[i].y, 1);
		Mat_<double> x2 = (Mat_<double>(3, 1) << pts2[i].x, pts2[i].y, 1);

		Mat_<double> xFx = x2.t() * F * x1;
		std::cout << xFx << std::endl;
		double Fx1_1 = F(0, 0) * pts1[i].x + F(0, 1) * pts1[i].y + F(0, 2);
		double Fx1_2 = F(1, 0) * pts1[i].x + F(1, 1) * pts1[i].y + F(1, 2);
		double Fx2_1 = F(0, 0) * pts2[i].x + F(1, 0) * pts2[i].y + F(2, 0);
		double Fx2_2 = F(0, 1) * pts2[i].x + F(1, 1) * pts2[i].y + F(2, 1);

		double denom = Fx1_1 * Fx1_1 + Fx1_2 * Fx1_2 + Fx2_1 * Fx2_1 + Fx2_2 * Fx2_2;

		pts1[i].x -= xFx(0, 0) / denom * Fx2_1;
		pts1[i].y -= xFx(0, 0) / denom * Fx2_2;
		pts2[i].x -= xFx(0, 0) / denom * Fx1_1;
		pts2[i].x -= xFx(0, 0) / denom * Fx1_2;
	}
}

bool Reconstruction::decomposeEtoRandT(const Mat_<double>& E, Mat_<double>& R1, Mat_<double>& R2, Mat_<double>& t1, Mat_<double>& t2) {
	SVD svd(E);

	//check if first and second singular values are the same (as they should be)
	double singular_values_ratio = fabsf(svd.w.at<double>(0) / svd.w.at<double>(1));
	if(singular_values_ratio>1.0) singular_values_ratio = 1.0/singular_values_ratio; // flip ratio to keep it [0,1]
	if (singular_values_ratio < 0.7) {
		std::cout << "singular values are too far apart\n";
		return false;
	}
	Matx33d W(0,-1,0, //HZ 9.13
			  1,0,0,
			  0,0,1);
	Matx33d Wt(0,1,0,
			  -1,0,0,
			   0,0,1);

	R1 = svd.u * Mat(W) * svd.vt; //HZ 9.19
	R2 = svd.u * Mat(Wt) * svd.vt; //HZ 9.19
	t1 = svd.u.col(2); //u3
	t2 = -svd.u.col(2); //u3

	return true;
}

void Reconstruction::calibrateCamera(std::vector<Mat>& img, Mat_<double>& K, Mat_<double> distCoeffs, std::vector<Mat>& P) {
	K = Mat_<double>::eye(3, 3);
	distCoeffs = Mat_<double>::zeros(1, 8);
	std::vector<Mat> rvecs;
	std::vector<Mat> tvecs;

	std::vector<std::vector<cv::Point3f> > objectPoints;
	objectPoints.resize(img.size());

	std::vector<std::vector<cv::Point2f> > pts;
	pts.resize(img.size());

	for (int i = 0; i < img.size(); ++i) {
		// ３Ｄ座標のセット
		for (int r = 0; r < 7; ++r) {
			for (int c = 0; c < 10; ++c) {
				objectPoints[i].push_back(cv::Point3f(c * 21.7, (6-r) * 21.7, 0.0f));
			}
		}

		// コーナー検出
		if (cv::findChessboardCorners(img[i], cv::Size(10, 7), pts[i])) {
			fprintf (stderr, "ok\n");
		} else {
			fprintf (stderr, "fail\n");
		}

		// サブピクセル精度のコーナー検出
		cv::Mat grayMat(img[i].size(), CV_8UC1);
		cv::cvtColor(img[i], grayMat, CV_RGB2GRAY);
		cv::cornerSubPix(grayMat, pts[i], cv::Size(3, 3), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03));

		// ファイルに保存する
		char filename[256];
		sprintf(filename, "corners%d.jpg", i);
		Mat temp_img = img[i].clone();
		for (int k = 0; k < pts[i].size(); ++k) {
			circle(temp_img, Point(pts[i][k].x, pts[i][k].y), 5, Scalar(255, 255, 255), 2);
		}
		imwrite(filename, temp_img);

		// Y座標を上下反転させる
		for (int j = 0; j < pts[i].size(); ++j) {
			pts[i][j].y = img[i].rows - pts[i][j].y;
		}
	}

	cv::calibrateCamera(objectPoints, pts, img[0].size(), K, distCoeffs, rvecs, tvecs, CV_CALIB_ZERO_TANGENT_DIST | CV_CALIB_FIX_K2 | CV_CALIB_FIX_K3 | CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6);

	P.resize(img.size());
	for (int i = 0; i < img.size(); ++i) {
		Mat_<double> R;
		Rodrigues(rvecs[i], R);

		P[i] = (Mat_<double>(3, 4) << R(0, 0), R(0, 1), R(0, 2), tvecs[i].at<double>(0, 0),
									  R(1, 0), R(1, 1), R(1, 2), tvecs[i].at<double>(1, 0),
									  R(2, 0), R(2, 1), R(2, 2), tvecs[i].at<double>(2, 0));
	}
}

void Reconstruction::bundleAdjustment(Mat_<double>& F, Mat_<double>& P1, Mat_<double>& P2, Mat_<double>& K, Point2f& pt1, Point2f& pt2, Point3d& pt3d) {
	// パラメータの数 ( 3 )
	const int NUM_PARAMS = 3;

	// データ数 (評価式の数 1 )
	int total_m = 1;

	// パラメータ (X, Y, Z)
	real x[NUM_PARAMS];

	// パラメータの初期推定値
	x[0] = pt3d.x;
	x[1] = pt3d.y;
	x[2] = pt3d.z;

	// 観測データ（ダミー）
	real y[1];

	// 真値と観測データとの誤差が格納される配列
	real fvec[1];

	// 結果のヤコビ行列
	real* fjac = new real[total_m * NUM_PARAMS];

	// lmdif内部使用パラメータ
	int ipvt[NUM_PARAMS];

	real diag[NUM_PARAMS], qtf[NUM_PARAMS], wa1[NUM_PARAMS], wa2[NUM_PARAMS], wa3[NUM_PARAMS];
	real* wa4 = new real[total_m * 2];

	// 観測データを格納する構造体オブジェクト
	fcndata_t data;
	data.m = total_m;
	data.y = y;
	data.pt3d = &pt3d;
	data.pt1 = &pt1;
	data.pt2 = &pt2;
	data.K = &K;
	data.P1 = &P1;
	data.P2 = &P2;

	// 観測データの数と同じ値にすることを推奨する
	int ldfjac = total_m * 2;

	// 各種パラメータ（推奨値のまま）
	real ftol = sqrt(__cminpack_func__(dpmpar)(1));
	real xtol = sqrt(__cminpack_func__(dpmpar)(1));
	real gtol = 0.;

	// 最大何回繰り返すか？
	int maxfev = 1000;

	// 収束チェック用の微小値
	real epsfcn = 1e-010;//1e-08;
	int mode = 1;

	// 1が推奨されている？
	real factor = 1;//1.e2;

	// 実際に繰り返した回数
	int nfev;

	int nprint = 0;
	int info = __cminpack_func__(lmdif)(fcn, &data, total_m, NUM_PARAMS, x, fvec, ftol, xtol, gtol, maxfev, epsfcn,
									diag, mode, factor, nprint, &nfev, fjac, ldfjac, ipvt, qtf, wa1, wa2, wa3, wa4);
	real fnorm = __cminpack_func__(enorm)(total_m, fvec);

	printf("final l2 norm of the residuals: %15.7g\n\n", (double)fnorm);
	printf("number of function evaluations: %10i\n\n", nfev);
	printf("exit parameter %10i\n\n", info);

	// 収束結果を反映する
	pt3d.x = x[0];
	pt3d.y = x[1];
	pt3d.z = x[2];

	// 真値と観測データの差を合計する
	/*
	double total_error = 0.0;
	for (int i = 0; i < 2; ++i) {
		std::vector<cv::Point2f> projectedImagePoints;
		
		projectPoints(objectPoints[i], rvecs[i], tvecs[i], cameraMat, distortion, projectedImagePoints);

		for (int j = 0; j < 70; ++j) {
			// 射影結果と観測データの誤差
			total_error += sqrt(SQR(projectedImagePoints[j].x - imagePoints[i][j].x) + SQR(projectedImagePoints[j].y - imagePoints[i][j].y));
		}
	}
	*/

	// メモリ解放
	delete [] fjac;
	delete [] wa4;
}

/**
 * 自分の関数を記述し、真値と観測データとの差を計算する。
 *
 * @param p		観測データが入った構造体オブジェクト
 * @param m		観測データの数 ( 点の数 )
 * @param n		パラメータの数 ( 3 x 点の数 )
 * @param x		パラメータ配列 ( (X, Y, Z) * 点の数 )
 * @param fvec	真値と観測データとの差を格納する配列
 * @param iflag	lmdifから返されるフラグ (0なら終了?)
 * @return		0を返却する
 */
int Reconstruction::fcn(void *p, int m, int n, const real *x, real *fvec, int iflag) {
	const real *y = ((fcndata_t*)p)->y;

	if (iflag == 0) {
		/* insert print statements here when nprint is positive. */
		/* if the nprint parameter to lmdif is positive, the function is
		called every nprint iterations with iflag=0, so that the
		function may perform special operations, such as printing
		residuals. */
		return 0;
	}

	Mat_<double> K = *(((fcndata_t*)p)->K);
	Mat_<double> P1 = *(((fcndata_t*)p)->P1);
	Mat_<double> P2 = *(((fcndata_t*)p)->P2);

	
	Mat_<double> X = (Mat_<double>(4, 1) << ((fcndata_t*)p)->pt3d->x, ((fcndata_t*)p)->pt3d->y, ((fcndata_t*)p)->pt3d->z, 1);
	Mat_<double> x1 = K * P1 * X;
	Mat_<double> x2 = K * P2 * X;
	Point2f u1(x1(0, 0) / x1(2, 0), x1(1, 0) / x1(2, 0));
	Point2f u2(x2(0, 0) / x2(2, 0), x2(1, 0) / x2(2, 0));

	fvec[0] = norm(u1 - *(((fcndata_t*)p)->pt1)) + norm(u2 - *(((fcndata_t*)p)->pt2));

	return 0;
}
