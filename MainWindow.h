#pragma once
#include <QMainWindow>
#include "ui_MainWindow.h"
#include <QTimer>
#include "MyGLWidget.h"
#include "DataProcessing.h"
#include "SurfaceReconsturction.h"
#include <QMutex>
class MainWindow : public QMainWindow{
	Q_OBJECT
public:
	MainWindow(QWidget *parent = nullptr);
	~MainWindow();
public slots:
	void chooseFile();
	void startRendering();
	void startUpdateGL();
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

	std::vector<QVector3D> rawData;
};	
