#version 450 core
layout (location = 0) in vec3 vertexPosition;
layout (location = 1) in vec3 vertexNormal;

out vec3 vPosition;
out vec3 vNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform vec3 brushPosition;
uniform float brushSize;
uniform bool isMouseBrushUsed = false;

void main(){

	vPosition = vec3(model * vec4(vertexPosition, 1.0));// 点的位置
	vNormal = mat3(transpose(inverse(model))) * vertexNormal;

	if(isMouseBrushUsed){
		vec3 delta = vPosition - brushPosition;
		float dist = length(delta);
		if (dist < brushSize) {
			float amount = 4.0 * (1.0 - dist / brushSize);
			vec3 direction = normalize(delta);
			vec3 offset = direction * amount;
			vPosition += offset;
		}
	}
	gl_Position = proj * view * vec4(vPosition, 1.0f);
}
 