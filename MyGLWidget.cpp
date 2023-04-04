#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <mutex>
#include <QMutex>
#include <glut.h>
QMutex m_mutex;
MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
}

MyGLWidget::~MyGLWidget(){
    delete pointShader;
    delete meshShader;
    glFunc->glDeleteVertexArrays(1, &meshVAO);
    glFunc->glDeleteBuffers(1, &meshVBO);
}

void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::setCameraPara(QVector3D eye, QVector3D center) {
    camera->eye = eye;
    camera->center = center;
}
void MyGLWidget::initializeShader() {
    QString qAppDir = QCoreApplication::applicationDirPath();
    QString pointVert = qAppDir + "/Shader/point.vert", pointFrag = qAppDir + "/Shader/point.frag";
    QString meshVert = qAppDir + "/Shader/mesh.vert", meshFrag = qAppDir + "/Shader/mesh.frag";

    pointShader = new ShaderProgram(pointVert.toStdString().c_str(), pointFrag.toStdString().c_str());
    meshShader = new ShaderProgram(meshVert.toStdString().c_str(), meshFrag.toStdString().c_str());
}

void MyGLWidget::initializeGL(){
    initializeShader();
    glFunc = QOpenGLContext::currentContext()->versionFunctions<QOpenGLFunctions_4_5_Core>();
    glFunc->glEnable(GL_DEPTH_TEST);
}
// PaintGL
void MyGLWidget::paintGL(){
    glFunc->glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glFunc->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (DataType::PointType == dataType) {
        glFunc->glPointSize(1.0f);
        glFunc->glGenVertexArrays(1, &pointVAO);
        glFunc->glBindVertexArray(pointVAO);
        glFunc->glGenBuffers(1, &pointVBO);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        
        pointShader->use();
        pointShader->setUniformMat4("model", model);
        pointShader->setUniformMat4("view", camera->getViewMatrix());
        pointShader->setUniformMat4("proj", proj);
        glFunc->glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
    }else {
        glFunc->glGenVertexArrays(1, &meshVAO);
        glFunc->glBindVertexArray(meshVAO);
        glFunc->glGenBuffers(1, &meshVBO);
        glFunc->glBindBuffer(GL_ARRAY_BUFFER, meshVBO);

        glFunc->glEnableVertexAttribArray(0);
        glFunc->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glFunc->glEnableVertexAttribArray(1);
        glFunc->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

        glFunc->glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        
        meshShader->use();
        meshShader->setUniformVec3("viewPos", QVector3D(0.0f, 0.0f, 3.0f));
        meshShader->setUniformVec3("material.ambient", QVector3D(0.5f, 0.5f, 0.5f));
        meshShader->setUniformVec3("material.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
        meshShader->setUniformVec3("material.specular", QVector3D(0.5f, 0.5f, 0.5f));
        meshShader->setUniformFloat("material.shininess", 16.0f);

        meshShader->setUniformVec3("dirLight1.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        meshShader->setUniformVec3("dirLight1.diffuse", QVector3D(0.5f, 0.5f, 0.9f));
        meshShader->setUniformVec3("dirLight1.specular", QVector3D(0.1f, 0.1f, 0.1f));
        meshShader->setUniformVec3("dirLight1.direction", QVector3D(1.0f, 1.0f, 3.0f));

        meshShader->setUniformVec3("dirLight2.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        meshShader->setUniformVec3("dirLight2.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
        meshShader->setUniformVec3("dirLight2.specular", QVector3D(0.1f, 0.1f, 0.1f));
        meshShader->setUniformVec3("dirLight2.direction", QVector3D(1.0f, 1.0f, -3.0f));

        meshShader->setUniformMat4("model", model);
        meshShader->setUniformMat4("view", camera->getViewMatrix());
        meshShader->setUniformMat4("proj", proj);

        glFunc->glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
    }
}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint p_ab = event->pos();
    translate_point(p_ab);
    QPoint subPoint = p_ab - pressPosition;

    model.setToIdentity();
    modelSave.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
        GLfloat angleNow = qSqrt(qPow(subPoint.x(), 2) + qPow(subPoint.y(), 2)) / 5;
        model.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
        model = model * modelUse;

        modelSave.rotate(angleNow, -subPoint.y(), subPoint.x(), 0.0);
        modelSave = modelSave * modelUse;
    }
    if (event->buttons() & Qt::RightButton) {
        model.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        model = model * modelUse;

        modelSave.translate((float)subPoint.x() / 200, (float)subPoint.y() / 200);
        modelSave = modelSave * modelUse;
    }
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    setPressPosition(event->pos());
    modelUse = modelSave;
    repaint();
}

void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {
    setPressPosition(event->pos());
    modelUse = modelSave;
}
void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QPoint offset = event->angleDelta();
    camera->mouseScroll(offset.y() / 20);
    repaint();
}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {

}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {

}

void MyGLWidget::setPressPosition(QPoint p_ab) {
    translate_point(p_ab);
    pressPosition = p_ab;
}
void MyGLWidget::translate_point(QPoint& p_ab) {
    int x = p_ab.x() - this->width() / 2;
    int y = -(p_ab.y() - this->height() / 2);
    p_ab = {x,y};
}