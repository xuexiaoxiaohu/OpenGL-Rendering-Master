#include "camera.h"
#include <QMatrix4x4>
Camera::Camera(QVector3D eye,QVector3D dir,QVector3D up):
	eye(eye),
	dir(dir),
	up(up),
	scrollSen(0.01f){}

QMatrix4x4 Camera::getViewMatrix(){
	QMatrix4x4 matrix;
	matrix.setToIdentity();
	matrix.lookAt(eye, dir, up);
	return matrix;
}
void Camera::mouseScroll(float yoffset){
	yoffset *= scrollSen;
	eye.setZ(eye.z() + yoffset);
}