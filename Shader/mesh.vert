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

void main(){
	vPosition = vertexPosition;
	vNormal = mat3(transpose(inverse(model))) * vertexNormal;
	gl_Position = proj * view * model * vec4(vPosition, 1.0f);
}
 