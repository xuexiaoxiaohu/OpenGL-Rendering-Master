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
	void openFile();
	void startRendering();
	void callbackRepaint();
signals:
	void signal_glUpdate();

private:
	Ui::MainWindowClass ui;
	void addOpengGLWidget();

	MyGLWidget* myPointGLWidget;
	MyGLWidget* myMeshGLWidget;

	SurfaceReconsturction* surface;
	DataProcessing* pointProc;
	DataProcessing* meshProc;

	std::vector<QVector3D> rawData;
	std::vector<QVector3D> glMeshVertices;
	std::vector<GLfloat> glPoint;
	std::vector<GLfloat> glMesh;
};	
