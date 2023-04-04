#pragma once
#include <QVector3D>

class Camera
{
public:
	QVector3D eye;
	QVector3D center;
	QVector3D up;
	float scrollSensitivity;

	Camera(QVector3D eye = QVector3D(0.0f, 0.0f, 3.0f),QVector3D center = QVector3D(0.0, 0.0, 0.0),QVector3D up = QVector3D(0.0f, 1.0f, 0.0f)){
		this->eye = eye;
		this->center = center;
		this->up = up;
		this->scrollSensitivity = 0.01f;
	}
	// Get 
	QMatrix4x4 getViewMatrix(); 
	// Mouse Wheel 
	void mouseScroll(float yoffset);
};

