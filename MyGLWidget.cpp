#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <mutex>
#include <QMutex>
#include <Macro.h>
#include <glut.h>
#include <QMessageBox>
#include <QPainter>
#include <vtkFillHolesFilter.h>
MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    glDataProc = new DataProcessing();
    projMatrix.setToIdentity();
    projMatrix.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
}

MyGLWidget::~MyGLWidget(){
    delete pointShader;
    delete meshShader;
    glFunc->glDeleteVertexArrays(1, &meshVAO);
    glFunc->glDeleteBuffers(1, &meshVBO);
}
void MyGLWidget::setMesh(pcl::PolygonMesh mesh) {
    this->mesh = mesh;
}
void MyGLWidget::setMeshVertices(std::vector<QVector3D> allVertices) {
    this->allVertices = allVertices;
}

void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::setCameraPara(QVector3D eye, QVector3D dir) {
    camera->eye = eye;
    camera->dir = dir;
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
    glFunc->glEnable(GL_BLEND);
    glFunc->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
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
        pointShader->setUniformMat4("model", modelMatrix);
        pointShader->setUniformMat4("view", camera->getViewMatrix());
        pointShader->setUniformMat4("proj", projMatrix);
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
            meshShader->setUniformVec3("mat.ambient", QVector3D(0.5f, 0.5f, 0.5f));
            meshShader->setUniformVec3("mat.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
            meshShader->setUniformVec3("mat.specular", QVector3D(0.5f, 0.5f, 0.5f));
            meshShader->setUniformFloat("mat.shininess", 16.0f);

            meshShader->setUniformVec3("dl1.ambient", QVector3D(0.2f, 0.2f, 0.2f));
            meshShader->setUniformVec3("dl1.diffuse", QVector3D(0.5f, 0.5f, 0.9f));
            meshShader->setUniformVec3("dl1.specular", QVector3D(0.1f, 0.1f, 0.1f));
            meshShader->setUniformVec3("dl1.direction", QVector3D(1.0f, 1.0f, 3.0f));

            meshShader->setUniformVec3("dl2.ambient", QVector3D(0.2f, 0.2f, 0.2f));
            meshShader->setUniformVec3("dl2.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
            meshShader->setUniformVec3("dl2.specular", QVector3D(0.1f, 0.1f, 0.1f));
            meshShader->setUniformVec3("dl2.direction", QVector3D(1.0f, 1.0f, -3.0f));

            meshShader->setUniformMat4("model", modelMatrix);
            meshShader->setUniformMat4("view", camera->getViewMatrix());
            meshShader->setUniformMat4("proj", projMatrix);

            glFunc->glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
    }
}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    mMousePos = event->pos();
    QVector3D diff = QVector3D(mMousePos - m_lastPos);
    float angle = diff.length() / 2.0f;
    QVector3D axis = QVector3D(diff.y(), diff.x(), 0.0f).normalized();

    rotationAngle += angle;
    rotationAxis = axis;
    m_lastPos = mMousePos;

    modelMatrix.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
        rotateMesh(rotationAngle, rotationAxis);
    }
    if (event->buttons() & Qt::RightButton) {
        modelMatrix.translate(mMousePos.x()/50, mMousePos.y()/50);
    }
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (isShiftPressed) {
        if (event->buttons() & Qt::LeftButton) {
            if (isConstructionFinished == false) {
                QMessageBox::information(this, "Tips", "Please perform the erase operation "
                    "after modeling is completed.", QMessageBox::Ok);
                return;
            }
            QVector3D worldPos = convertScreenToWorld(event->pos());
            glDataProc->getErasedMesh(worldPos, mesh, allVertices);
 
            vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
            pcl::io::mesh2vtk(mesh, polydata);
   
            vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();
            fillHolesFilter->SetInputData(polydata);
            fillHolesFilter->SetHoleSize(100.0);
            fillHolesFilter->Update();
            pcl::io::vtk2mesh(fillHolesFilter->GetOutput(), mesh);
            glDataProc->getGLMeshData(mesh);

            setImageData(glDataProc->glMeshData);
        }
    }else{
        if (event->buttons() & Qt::LeftButton) {
            m_lastPos = event->pos();
        }
    }
    repaint();
}
void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {


}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) {
        isShiftPressed = true;
    }
}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) {
        isShiftPressed = false;
    }
}
void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QPoint offset = event->angleDelta();
    camera->mouseScroll(offset.y());
    repaint();
}
QVector3D MyGLWidget::convertScreenToWorld(QPoint sp) {
    int viewport[4] = { 0, 0, SCR_WIDTH, SCR_HEIGHT };
    double mvMatrix[16], pMatrix[16];

    QMatrix4x4 modelViewMatrix = (camera->getViewMatrix()) * modelMatrix;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mvMatrix[i * 4 + j] = modelViewMatrix(j, i);
            pMatrix[i * 4 + j] = projMatrix(j, i);
        }
    }
    GLfloat depth;
    double wx, wy, wz;
    makeCurrent();
    glReadPixels(sp.x(), viewport[3] - sp.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    gluUnProject(sp.x(), viewport[3] - sp.y(), depth, mvMatrix, pMatrix, viewport, &wx, &wy, &wz);
    return QVector3D((double)wx, (double)wy, (double)wz);
}
void MyGLWidget::rotateMesh(float angle, QVector3D axis) {
    modelMatrix.translate(camera->dir);
    modelMatrix.rotate(angle, axis);
    modelMatrix.translate(-camera->dir);
}