#include "MainWindow.h"

MainWindow::MainWindow(QWidget *parent, Qt::WFlags flags) : QMainWindow(parent, flags) {
	ui.setupUi(this);

	connect(ui.actionFeatureExtraction, SIGNAL(triggered()), this, SLOT(onFeatureExtraction()));
	connect(ui.actionCalibration, SIGNAL(triggered()), this, SLOT(onCalibration()));
	connect(ui.actionReconstruction, SIGNAL(triggered()), this, SLOT(onReconstruction()));
	connect(ui.actionExit, SIGNAL(triggered()), this, SLOT(close()));

	connect(ui.actionRenderTexture, SIGNAL(triggered()), this, SLOT(onRenderTexture()));
	connect(ui.actionRenderWireframe, SIGNAL(triggered()), this, SLOT(onRenderWireframe()));


	// setup the OpenGL widget
	glWidget = new GLWidget3D(this);
	setCentralWidget(glWidget);
}

void MainWindow::onFeatureExtraction() {
	std::vector<Mat> img(2);
	img[0] = cv::imread("images/image1.jpg");
	img[1] = cv::imread("images/image2.jpg");

	glWidget->featureExtraction(img);
}

void MainWindow::onCalibration() {
    QFileDialog dialog(this);
    dialog.setDirectory(QDir::homePath());
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(trUtf8("Image files (*.jpg *.png)"));
    QStringList fileNames;
    if (dialog.exec()) {
	    fileNames = dialog.selectedFiles();
		std::vector<Mat> img(fileNames.size());

		for (int i = 0; i < fileNames.size(); ++i) {
			img[i] = cv::imread(fileNames[i].toUtf8().data());
		}

		glWidget->calibrateCamera(img);
	}
}

void MainWindow::onReconstruction() {
	glWidget->reconstruct();
}

void MainWindow::onRenderTexture() {
	ui.actionRenderWireframe->setChecked(false);
	glWidget->renderTexture();
}

void MainWindow::onRenderWireframe() {
	ui.actionRenderTexture->setChecked(false);
	glWidget->renderWireframe();
}
