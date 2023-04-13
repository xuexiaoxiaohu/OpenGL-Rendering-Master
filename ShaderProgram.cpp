#include "ShaderProgram.h"
#include <QOpenGLShader>
#include <string>
#include <fstream>
#include <sstream>

ShaderProgram::ShaderProgram(const char* vPath, const char* fPath){
	std::fstream vShaderFile(vPath), fShaderFile(fPath);
	if (vShaderFile.is_open() == NULL || 
		fShaderFile.is_open() == NULL) return;

	std::stringstream vShaderStream, fShaderStream;
	vShaderStream << vShaderFile.rdbuf();
	fShaderStream << fShaderFile.rdbuf();

	vShaderFile.close();
	fShaderFile.close();

	QOpenGLShader vertShader(QOpenGLShader::Vertex);
	QOpenGLShader fragShader(QOpenGLShader::Fragment);

	vertShader.compileSourceCode(vShaderStream.str().c_str());
	fragShader.compileSourceCode(fShaderStream.str().c_str());

	shaderProgram = new QOpenGLShaderProgram();
	shaderProgram->addShader(&vertShader);
	shaderProgram->addShader(&fragShader);
	shaderProgram->link();
}
void ShaderProgram::use()
{
	shaderProgram->bind();
}
ShaderProgram::~ShaderProgram()
{
	delete shaderProgram;
}
void ShaderProgram::setUniformMat4(const char* name, const QMatrix4x4 trans) const
{
	shaderProgram->setUniformValue(name, trans);
}
void ShaderProgram::setUniformMat3(const char* name, const QMatrix3x3 trans) const
{
	shaderProgram->setUniformValue(name, trans);
}
void ShaderProgram::setUniformFloat(const char* name, const GLfloat value) const
{
	shaderProgram->setUniformValue(name, value);
}
void ShaderProgram::setUniformInt(const char* name, const int value) const
{
	shaderProgram->setUniformValue(name, value);
}
void ShaderProgram::setUniformVec4(const char* name, const QVector4D value) const
{
	shaderProgram->setUniformValue(name, value);
}
void ShaderProgram::setUniformVec3(const char* name, const QVector3D value) const
{
	shaderProgram->setUniformValue(name, value);
}