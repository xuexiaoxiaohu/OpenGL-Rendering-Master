#pragma once
#include <QVector3D>

class Camera
{
public:
	QVector3D eye;
	QVector3D dir;
	QVector3D up;
	float scrollSensitivity;

	Camera(QVector3D eye = QVector3D(0.0f, 0.0f, 3.0f),
		QVector3D dir = QVector3D(0.0, 0.0, 0.0),
		QVector3D up = QVector3D(0.0f, 1.0f, 0.0f));
	// Get 
	QMatrix4x4 getViewMatrix(); 
	// Mouse Wheel 
	void mouseScroll(float yoffset);
};

