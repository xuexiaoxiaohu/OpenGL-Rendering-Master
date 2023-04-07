#include "MyGLWidget.h"
#include <QCoreApplication>
#include <iostream>
#include <mutex>
#include <QMutex>
#include <Macro.h>
#include <glut.h>
#include <QMessageBox>
#include <QPainter>

MyGLWidget::MyGLWidget(QWidget* parent,int DT){
    dataType = DT;
    camera = new Camera();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
    dataProc = new DataProcessing();
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
void MyGLWidget::setCameraPara(QVector3D eye, QVector3D meshCenter) {
    camera->eye = eye;
    camera->meshCenter = meshCenter;
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
        pointShader->setUniformMat4("model", model);
        pointShader->setUniformMat4("view", camera->getViewMatrix());
        pointShader->setUniformMat4("proj", proj);
        glFunc->glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
    }else {
        if (isShiftPressed){
            glColor3f(1.0f, 0.0f, 0.0f);
            QVector<float> vertices = dataProc->getCircularVertex(mMousePos, 0.05f, width(), height());

            glEnableClientState(GL_VERTEX_ARRAY);
            glVertexPointer(2, GL_FLOAT, 0, vertices.constData());

            glDrawArrays(GL_TRIANGLE_FAN, 0, vertices.size() / 2);
            glDisableClientState(GL_VERTEX_ARRAY);
        }else{
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
}
void MyGLWidget::resizeGL(int width, int height){
    glFunc->glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    mMousePos = event->pos();
    if (isShiftPressed){
        if (isConstructionFinished == false) {
            QMessageBox::information(this, "Tips", "Please perform the erase operation"
                "after modeling is completed.", QMessageBox::Ok);
            return;
        }
        QVector3D worldPos = convertScreenToWorld(mMousePos);
        dataProc->getDataAfterErase(worldPos, mesh, allVertices);

        setImageData(dataProc->glMeshData);

    }else{
        QVector3D diff = QVector3D(mMousePos - m_lastPos);
        float angle = diff.length() / 2.0f;
        QVector3D axis = QVector3D(diff.y(), diff.x(), 0.0f).normalized();

        rotationAngle += angle;
        rotationAxis = axis;
        m_lastPos = mMousePos;

        model.setToIdentity();
        if (event->buttons() & Qt::LeftButton) {
            model.rotate(rotationAngle, -axis.y(), axis.x(), 0.0);
        }
        if (event->buttons() & Qt::RightButton) {

        }
    }
   // repaint that both mesh "erase" and mesh "rotate and translate" operations. 
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    QPoint mousePos = event->pos();
    if (event->buttons() & Qt::LeftButton) {
   
    }
    else if (event->buttons() & Qt::LeftButton) {
        m_lastPos = event->pos();
    }
    repaint();
}
void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {


}
void MyGLWidget::wheelEvent(QWheelEvent* event) {
    QPoint offset = event->angleDelta();
    camera->mouseScroll(offset.y());
    repaint();
}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift)  isShiftPressed = true;
}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() == Qt::Key_Shift)  isShiftPressed = false;
}

QVector3D MyGLWidget::convertScreenToWorld(QPoint screenPoint) {
    int viewport[4] = { 0, 0, SCR_WIDTH, SCR_HEIGHT };
    double modelViewMatrix[16];
    double projectionMatrix[16];

    QMatrix4x4 mVMatrix = (camera->getViewMatrix()) * model;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            modelViewMatrix[i * 4 + j] = mVMatrix(j, i);
            projectionMatrix[i * 4 + j] = proj(j, i);
        }
    }
    GLfloat depthValue;
    makeCurrent();
    glReadPixels(screenPoint.x(), viewport[3] - screenPoint.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depthValue);
    doneCurrent();

    double worldX, worldY, worldZ;
    gluUnProject(screenPoint.x(), viewport[3] - screenPoint.y(), depthValue, modelViewMatrix, projectionMatrix, viewport, &worldX, &worldY, &worldZ);
    return QVector3D((double)worldX, (double)worldY, (double)worldZ);
}