#version 450 core
layout (location = 0) in vec3 vertexPos;
layout (location = 1) in vec3 vertexNormal;

out vec3 vNormal;
out vec3 vPosition;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform vec3 brushPosition;
uniform float brushSize;

void main(){
	vNormal = mat3(transpose(inverse(model))) * vertexNormal;
	vPosition = vertexPos;
	gl_Position = proj * view * model * vec4(vPosition, 1.0f);
}
 