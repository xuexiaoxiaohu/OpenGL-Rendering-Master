#version 450 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

out vec3 worldPos;  
out vec3 normal;
out vec3 vPosition;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

uniform vec3 brushPosition;
uniform float brushSize;

void main(){

	worldPos = vec3(model * vec4(aPos, 1.0));
	normal = mat3(transpose(inverse(model))) * aNormal;
	vPosition = aPos;
	gl_Position = proj * view * model * vec4(vPosition, 1.0f);
}
 