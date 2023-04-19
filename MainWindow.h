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
	void updateCursor();

signals:
	void signal_glUpdate();

private:
	Ui::MainWindowClass ui;
	void addOpengGLWidget();

	MyGLWidget* mPointGLWidget;
	MyGLWidget* mMeshGLWidget;

	SurfaceReconsturction* surface;
	DataProcessing* pointProc;
	DataProcessing* meshProc;
};	
