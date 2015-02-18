/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created: Wed Feb 18 10:17:21 2015
**      by: Qt User Interface Compiler version 4.8.2
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindowClass
{
public:
    QAction *actionExit;
    QAction *actionReconstruction;
    QAction *actionCalibration;
    QAction *actionFeatureExtraction;
    QAction *actionRenderTexture;
    QAction *actionRenderWireframe;
    QWidget *centralWidget;
    QMenuBar *menuBar;
    QMenu *menuFile;
    QMenu *menuRendering;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindowClass)
    {
        if (MainWindowClass->objectName().isEmpty())
            MainWindowClass->setObjectName(QString::fromUtf8("MainWindowClass"));
        MainWindowClass->resize(600, 400);
        actionExit = new QAction(MainWindowClass);
        actionExit->setObjectName(QString::fromUtf8("actionExit"));
        actionReconstruction = new QAction(MainWindowClass);
        actionReconstruction->setObjectName(QString::fromUtf8("actionReconstruction"));
        actionCalibration = new QAction(MainWindowClass);
        actionCalibration->setObjectName(QString::fromUtf8("actionCalibration"));
        actionFeatureExtraction = new QAction(MainWindowClass);
        actionFeatureExtraction->setObjectName(QString::fromUtf8("actionFeatureExtraction"));
        actionRenderTexture = new QAction(MainWindowClass);
        actionRenderTexture->setObjectName(QString::fromUtf8("actionRenderTexture"));
        actionRenderTexture->setCheckable(true);
        actionRenderTexture->setChecked(true);
        actionRenderWireframe = new QAction(MainWindowClass);
        actionRenderWireframe->setObjectName(QString::fromUtf8("actionRenderWireframe"));
        actionRenderWireframe->setCheckable(true);
        centralWidget = new QWidget(MainWindowClass);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        MainWindowClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindowClass);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 600, 21));
        menuFile = new QMenu(menuBar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuRendering = new QMenu(menuBar);
        menuRendering->setObjectName(QString::fromUtf8("menuRendering"));
        MainWindowClass->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindowClass);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        MainWindowClass->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindowClass);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindowClass->setStatusBar(statusBar);

        menuBar->addAction(menuFile->menuAction());
        menuBar->addAction(menuRendering->menuAction());
        menuFile->addAction(actionFeatureExtraction);
        menuFile->addAction(actionCalibration);
        menuFile->addAction(actionReconstruction);
        menuFile->addSeparator();
        menuFile->addAction(actionExit);
        menuRendering->addAction(actionRenderTexture);
        menuRendering->addAction(actionRenderWireframe);

        retranslateUi(MainWindowClass);

        QMetaObject::connectSlotsByName(MainWindowClass);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindowClass)
    {
        MainWindowClass->setWindowTitle(QApplication::translate("MainWindowClass", "MainWindow", 0, QApplication::UnicodeUTF8));
        actionExit->setText(QApplication::translate("MainWindowClass", "Exit", 0, QApplication::UnicodeUTF8));
        actionReconstruction->setText(QApplication::translate("MainWindowClass", "3D Reconstruction", 0, QApplication::UnicodeUTF8));
        actionCalibration->setText(QApplication::translate("MainWindowClass", "Calibration", 0, QApplication::UnicodeUTF8));
        actionFeatureExtraction->setText(QApplication::translate("MainWindowClass", "Feature Extraction", 0, QApplication::UnicodeUTF8));
        actionRenderTexture->setText(QApplication::translate("MainWindowClass", "Texture", 0, QApplication::UnicodeUTF8));
        actionRenderWireframe->setText(QApplication::translate("MainWindowClass", "Wireframe", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("MainWindowClass", "File", 0, QApplication::UnicodeUTF8));
        menuRendering->setTitle(QApplication::translate("MainWindowClass", "Rendering", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindowClass: public Ui_MainWindowClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
