#include "camera.h"
#include <QMatrix4x4>
QMatrix4x4 Camera::getViewMatrix(){
	QMatrix4x4 matrix;
	matrix.lookAt(eye, dir, up);
	return matrix;
}
void Camera::mouseScroll(float yoffset){
	yoffset *= scrollSensitivity;
	eye.setZ(eye.z() + yoffset);
}