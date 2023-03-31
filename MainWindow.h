#pragma once
#include <QMainWindow>
#include "ui_MainWindow.h"
#include <QTimer>
#include "MyGLWidget.h"
#include "DataProcessing.h"
#include "SurfaceReconsturction.h"
#include <QMutex>
//#include <QMutexLocker>
class MainWindow : public QMainWindow{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();
	bool isReConstru = true;
public slots:
	void chooseFile();
	void startRendering();

signals:
	void signal_glUpdate();

private:
	Ui::MainWindowClass ui;
	void addOpengGLWidget();

	MyGLWidget* myPointGLWidget;
	MyGLWidget* myMeshGLWidget;

	SurfaceReconsturction* surface;
	DataProcessing* pointDataProc;
	DataProcessing* meshDataProc;

	std::vector<GLfloat> pointData;	
	std::vector<GLfloat> meshData;

	std::vector<QVector3D> originalPointData;
	std::vector<QVector3D>	pointData3D;
	bool isOpenGLThreadStart;
	std::ofstream fs;
};	
