#pragma once

#include <QGLWidget>
#include <QMouseEvent>
#include <QKeyEvent>
#include "Camera.h"
#include <QVector3D>
#include <vector>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

class MainWindow;

class GLWidget3D : public QGLWidget {
private:
	MainWindow* mainWin;
	Camera camera;
	QPoint lastPos;

	std::vector<cv::Mat> img;
	cv::Mat P[2];
	std::vector<std::vector<cv::Point2f> > pts;
	std::vector<cv::Point3d> pts3d;
	QMap<QString, std::pair<GLuint, std::vector<Point2f> > > textures;

public:
	GLWidget3D(MainWindow* mainWin);
	void drawScene();
	QVector2D mouseTo2D(int x,int y);
	void drawTriangle(int index1, int index2, int index3);
	GLuint generateTexture(int index1, int index2, int index3, std::vector<Point2f>& texCoord);
	void featureExtraction(std::vector<cv::Mat>& img);
	void reconstruct();
	void calibrateCamera(std::vector<cv::Mat>& img);
	int findPointIndex(std::vector<Point2f>& pts, Point2f& pt);

protected:
	void initializeGL();
	void resizeGL(int width, int height);
	void paintGL();    
	void mousePressEvent(QMouseEvent *e);
	void mouseMoveEvent(QMouseEvent *e);
	void mouseReleaseEvent(QMouseEvent *e);

};

