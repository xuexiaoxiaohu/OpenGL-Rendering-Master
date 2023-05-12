//OpenGL
#include <glut.h>
// Qt
#include <QCoreApplication>
#include <QMessageBox>
#include <QPixmap>
#include <QPainter>
#include <QTimer>
// Custom
#include "MyGLWidget.h"
#include <Macro.h>

MyGLWidget::MyGLWidget(QWidget* parent,int dataType):
    rotationAngle(0.0f),dataType(dataType), isShiftPressed(false){
    camera = new Camera();
    glDataProc = new DataProcessing();
    pShader = new QOpenGLShaderProgram();
    mShader = new QOpenGLShaderProgram();

    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
    
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
    timer->start(100);
}
void MyGLWidget::updateCursor() {
    if (isShiftPressed && isMouseBrush){
        QPixmap pixmap(brushSize, brushSize);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        //painter.setPen(Qt::red);
        painter.setBrush(Qt::red);
        painter.drawEllipse(0, 0, brushSize, brushSize);
        QCursor cursor(pixmap);
        setCursor(cursor);
    }else{
        setCursor(Qt::ArrowCursor);
    }
}
MyGLWidget::~MyGLWidget(){
    delete pShader;
    delete mShader;
    glDeleteVertexArrays(1, &mVAO);
    glDeleteBuffers(1, &mVBO);
}
void MyGLWidget::setMesh(pcl::PolygonMesh mesh) {
    this->mesh = mesh;
}
void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::setAdaptivePara(QVector3D center, float radius){
    camera->center = center;
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.01f, 20 * radius);
}

void MyGLWidget::initializeGL(){
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    QString qAppDir = QCoreApplication::applicationDirPath();
    QString pointVert = qAppDir + "/Shader/point.vert", pointFrag = qAppDir + "/Shader/point.frag";
    pShader->addShaderFromSourceFile(QOpenGLShader::Vertex, pointVert);
    pShader->addShaderFromSourceFile(QOpenGLShader::Fragment, pointFrag);
    pShader->link();
    QString meshVert = qAppDir + "/Shader/mesh.vert", meshFrag = qAppDir + "/Shader/mesh.frag";
    mShader->addShaderFromSourceFile(QOpenGLShader::Vertex, meshVert);
    mShader->addShaderFromSourceFile(QOpenGLShader::Fragment, meshFrag);
    mShader->link();
}
// PaintGL
void MyGLWidget::paintGL(){
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (DataType::PointType == dataType) {
        glPointSize(1.0f);
        glGenVertexArrays(1, &pVAO);
        glBindVertexArray(pVAO);
        glGenBuffers(1, &pVBO);
        glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);
        
        pShader->bind();
        pShader->setUniformValue("model", model);
        pShader->setUniformValue("view", camera->getViewMatrix());
        pShader->setUniformValue("proj", proj);

        glDrawArrays(GL_POINTS, 0, vertices.size() / 3);
    }else{
        glGenVertexArrays(1, &mVAO);
        glBindVertexArray(mVAO);
        glGenBuffers(1, &mVBO);
        glBindBuffer(GL_ARRAY_BUFFER, mVBO);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));

        glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        mShader->bind();
        mShader->setUniformValue("viewPos", QVector3D(0.0f, 0.0f, 3.0f));

        mShader->setUniformValue("mat.ambient", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformValue("mat.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformValue("mat.specular", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformValue("mat.shininess", 16.0f);

        mShader->setUniformValue("dl1.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        mShader->setUniformValue("dl1.diffuse", QVector3D(0.5f, 0.5f, 0.9f));
        mShader->setUniformValue("dl1.specular", QVector3D(0.1f, 0.1f, 0.1f));
        mShader->setUniformValue("dl1.direction", QVector3D(1.0f, 1.0f, 3.0f));

        mShader->setUniformValue("dl2.ambient", QVector3D(0.2f, 0.2f, 0.2f));
        mShader->setUniformValue("dl2.diffuse", QVector3D(0.5f, 0.5f, 0.5f));
        mShader->setUniformValue("dl2.specular", QVector3D(0.1f, 0.1f, 0.1f));
        mShader->setUniformValue("dl2.direction", QVector3D(1.0f, 1.0f, -3.0f));

        mShader->setUniformValue("model", model);
        mShader->setUniformValue("view", camera->getViewMatrix());
        mShader->setUniformValue("proj", proj);  

        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
    }
}
void MyGLWidget::resizeGL(int width, int height){
    glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    QPoint currentMousePos = event->pos();
    QVector3D diff = QVector3D(currentMousePos - lastMousePos);
    rotationAngle += (diff.length() / 2.0f);
    QVector3D rotationAxis = QVector3D(diff.y(), diff.x(), 0.0f).normalized();
    model.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
        if (isShiftPressed) {
            int deltaX = currentMousePos.x() - lastMousePos.x();
            int deltaY = currentMousePos.y() - lastMousePos.y();
            int  distance = sqrt(deltaX * deltaX + deltaY * deltaY);
            if (distance >= 2.0) {
                lastMousePos = currentMousePos;
                GLdouble wx, wy, wz;
                convScreen2World(currentMousePos, wx, wy, wz);
                glDataProc->getErasedMesh(QVector3D(wx, wy, wz), mesh, 0.1 * brushSize);
                setImageData(glDataProc->glMeshData);
            }
        }else{
           model.translate(camera->center);
           model.rotate(rotationAngle, rotationAxis);
           model.translate(-camera->center);
        }
    }
    if (event->buttons() & Qt::RightButton) {
        model.translate(currentMousePos.x()/20, currentMousePos.y()/20);
    }
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    if (event->buttons() & Qt::LeftButton) {
        lastMousePos = event->pos();
    }
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
    if (isShiftPressed){
        brushParam += (offset.y() * SCROLL_SEN);
        if (abs(brushParam) <= 0.0001) brushParam = 0.0f;
        if (brushParam > BRUSH_PARAM_MAX)  brushParam = BRUSH_PARAM_MAX;
        if (brushParam < BRUSH_PARAM_MIN) brushParam = BRUSH_PARAM_MIN;
        brushSize = PARAM_A * brushParam + PARAM_B;
    }else{
        camera->mouseScroll(offset.y());
        repaint();
    }
}
void MyGLWidget::convScreen2World(QPoint screenPoint, GLdouble& wx, GLdouble& wy, GLdouble& wz) {
    int viewport[4] = { 0, 0, SCR_WIDTH, SCR_HEIGHT };
    double mvArray[16], pArray[16];

    QMatrix4x4 mvMat = (camera->getViewMatrix()) * model;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            mvArray[i * 4 + j] = mvMat(j, i);
            pArray[i * 4 + j] = proj(j, i);
        }
    }

    GLfloat depth;
    makeCurrent();
    glReadPixels(screenPoint.x(), viewport[3] - screenPoint.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    gluUnProject(screenPoint.x(), viewport[3] - screenPoint.y(), depth, mvArray, pArray, viewport, &wx, &wy, &wz);
}