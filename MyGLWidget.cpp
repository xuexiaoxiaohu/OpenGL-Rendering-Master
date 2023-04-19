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
    rotationAngle(0.0f),
    rotationAxis(0.0f, 1.0f, 0.0f),
    dataType(dataType), 
    isShiftPressed(false)
{
    camera = new Camera();
    glDataProc = new DataProcessing();
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f, 200.f);
    this->grabKeyboard();
    
    QPixmap pixmap(32, 32);
    pixmap.fill(Qt::transparent);
    QPainter painter(&pixmap);
    painter.setPen(Qt::red);
    painter.setBrush(Qt::red);
    painter.drawEllipse(0, 0, 32, 32);
    QCursor cursor(pixmap);
    setCursor(cursor);

    QTimer* timer = new QTimer(this);
    timer->start(10);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
}
void MyGLWidget::updateCursor() {
    if (isShiftPressed){
        if (isMouseBrush) {
            QPixmap pixmap(32, 32);
            pixmap.fill(Qt::transparent);
            QPainter painter(&pixmap);
            painter.setPen(Qt::red);
            painter.setBrush(Qt::red);
            painter.drawEllipse(0, 0, 32, 32);
            QCursor cursor(pixmap);
            setCursor(cursor);
        }
    }
    else{
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
void MyGLWidget::setMeshVtx(std::vector<QVector3D> allVertices) {
    this->allVertices = allVertices;
}

void MyGLWidget::setImageData(std::vector<GLfloat> data){
    vertices = data;
}
void MyGLWidget::setAdaptivePara(QVector3D center, float radius){
    camera->dir = center;
    proj.setToIdentity();
    proj.perspective(45.0f, width() / height(), 0.1f * radius, 10.0f * radius);
}

void MyGLWidget::initializeGL(){
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    QString qAppDir = QCoreApplication::applicationDirPath();

    QString pointVert = qAppDir + "/Shader/point.vert", pointFrag = qAppDir + "/Shader/point.frag";
    pShader = new QOpenGLShaderProgram();
    pShader->addShaderFromSourceFile(QOpenGLShader::Vertex, pointVert);
    pShader->addShaderFromSourceFile(QOpenGLShader::Fragment, pointFrag);
    pShader->link();

    QString meshVert = qAppDir + "/Shader/mesh.vert", meshFrag = qAppDir + "/Shader/mesh.frag";
    mShader = new QOpenGLShaderProgram();
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

        mShader->setUniformValue("brushSize", brushSize);
        mShader->setUniformValue("brushPosition", brushPosition);

        GLint posAttrib = glGetAttribLocation(mShader->programId(),"brushSize");

        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
    }
}
void MyGLWidget::resizeGL(int width, int height){
    glViewport(0, 0, width, height);
}

void MyGLWidget::mouseMoveEvent(QMouseEvent* event){
    mMousePos = event->pos();
    QVector3D diff = QVector3D(mMousePos - m_lastPos);
    float angle = diff.length() / 2.0f;
    QVector3D rotationAxis = QVector3D(diff.y(), diff.x(), 0.0f).normalized();

    rotationAngle += angle;
    m_lastPos = mMousePos;

    model.setToIdentity();
    if (event->buttons() & Qt::LeftButton) {
        if (isShiftPressed) {
            //if (isConstrFin == false) {
            //    QMessageBox::information(this, "Tips", "Please perform the erase operation "
            //        "after modeling is completed.", QMessageBox::Ok);
            //    return;
            //}
            GLdouble wx, wy, wz;
            convScreen2World(mMousePos, wx, wy, wz);
            brushPosition = QVector3D(wx, wy, wz);

            //GLdouble wx, wy, wz;
            //convScreen2World(mMousePos, wx, wy, wz);
            //glDataProc->getErasedMesh(QVector3D(wx, wy, wz), mesh, allVertices);
            //setImageData(glDataProc->glMeshData);
        }
        else {
            rotateMesh(rotationAngle, rotationAxis);
        }
    }
    if (event->buttons() & Qt::RightButton) {
        model.translate(mMousePos.x() / 50, mMousePos.y() / 50);
    }
    repaint();
}
void MyGLWidget::mousePressEvent(QMouseEvent* event){
    QPoint pressPos = event->pos();
    if (event->buttons() & Qt::LeftButton) {
        m_lastPos = pressPos;
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
void MyGLWidget::convScreen2World(QPoint sp, GLdouble& wx, GLdouble& wy, GLdouble& wz) {
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
    glReadPixels(sp.x(), viewport[3] - sp.y(), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);
    gluUnProject(sp.x(), viewport[3] - sp.y(), depth, mvArray, pArray, viewport, &wx, &wy, &wz);
}
void MyGLWidget::rotateMesh(float angle, QVector3D axis) {
    model.translate(camera->dir);
    model.rotate(angle, axis);
    model.translate(-camera->dir);
}
void MyGLWidget::translatePoint(QPoint& pressPos) {
    int x = pressPos.x() - this->width() / 2;
    int y = -(pressPos.y() - this->height() / 2);
    pressPos = {x,y};
}