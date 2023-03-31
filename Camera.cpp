#include "camera.h"
#include <QMatrix4x4>
QMatrix4x4 Camera::getViewMatrix(){
	QMatrix4x4 matrix;
	matrix.lookAt(position, front, up);
	return matrix;
}
void Camera::mouseScroll(float yoffset){
	yoffset *= scrollSensitivity;
	position.setZ(position.z() + yoffset);
}