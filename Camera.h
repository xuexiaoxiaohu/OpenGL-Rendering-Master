#pragma once
#include <QVector3D>

class Camera
{
public:
	QVector3D position;
	QVector3D front;
	QVector3D up;

	float scrollSensitivity;

	Camera(QVector3D position = QVector3D(0.0f, 0.0f, 3.0f),QVector3D front = QVector3D(0.0, 0.0, 0.0),QVector3D up = QVector3D(0.0f, 1.0f, 0.0f)){
		this->position = position;
		this->up = up;
		this->front = front;
		this->scrollSensitivity = 0.01f;
	}
	// Get 
	QMatrix4x4 getViewMatrix(); 
	// Mouse Wheel 
	void mouseScroll(float yoffset);
};

