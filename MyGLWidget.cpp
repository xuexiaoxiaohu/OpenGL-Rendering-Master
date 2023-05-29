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
// lr 
#include <vtkImplicitSelectionLoop.h>
#include <vtkSelectPolyData.h>

MyGLWidget::MyGLWidget(QWidget* parent,int dataType)
    : rotationAngle(0.0f)
    , dataType(dataType)
    , isShiftPressed(false)
    , grayValue(0.5f)
    , brushPosition(0.0f,0.0f,-0.5f)
    , brushSize(4)
    , isMouseBrush(false)
{
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

        QVector3D grayValue3D(grayValue, grayValue,grayValue);
        QVector3D dirLight1(1.0f, 1.0f, 3.0f), dirLight2(1.0f, 1.0f, -3.0f);

        mShader->setUniformValue("mtrl.ambient", grayValue3D);
        mShader->setUniformValue("mtrl.diffuse", grayValue3D);
        mShader->setUniformValue("mtrl.specular", grayValue3D);
        mShader->setUniformValue("mtrl.shininess", 16.0f);

        mShader->setUniformValue("dirLight1.ambient", grayValue3D);
        mShader->setUniformValue("dirLight1.diffuse", grayValue3D);
        mShader->setUniformValue("dirLight1.specular", grayValue3D);
        mShader->setUniformValue("dirLight1.direction", dirLight1);

        mShader->setUniformValue("dirLight2.ambient", grayValue3D);
        mShader->setUniformValue("dirLight2.diffuse", grayValue3D);
        mShader->setUniformValue("dirLight2.specular", grayValue3D);
        mShader->setUniformValue("dirLight2.direction", dirLight2);

        mShader->setUniformValue("model", model);
        mShader->setUniformValue("view", camera->getViewMatrix());
        mShader->setUniformValue("proj", proj);  

        glDrawArrays(GL_TRIANGLES, 0, vertices.size() / 6);
        
        //glPointSize(5.0f);
        //glGenVertexArrays(1, &pVAO);
        //glBindVertexArray(pVAO);
        //glGenBuffers(1, &pVBO);
        //glBindBuffer(GL_ARRAY_BUFFER, pVBO);
        //glEnableVertexAttribArray(0);
        //glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        //glBufferData(GL_ARRAY_BUFFER, sizeof(GLfloat) * vertices.size(), vertices.data(), GL_DYNAMIC_DRAW);

        //pShader->bind();
        //pShader->setUniformValue("model", model);
        //pShader->setUniformValue("view", camera->getViewMatrix());
        //pShader->setUniformValue("proj", proj);

        //glDrawArrays(GL_POINTS, 0, vertices.size() / 3);        
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
    if (event->buttons() & Qt::LeftButton) 
        lastMousePos = event->pos();

    // lr
    if (m_record_poly_clip && event->button() == Qt::LeftButton)
    {
        //记录当前鼠标的位置；
        QPoint pos = event->pos();
        m_points.append(pos);
    }
}
void MyGLWidget::mouseReleaseEvent(QMouseEvent* event) {

}
void MyGLWidget::keyPressEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) 
        isShiftPressed = true;
    // lr
    // A 键启动多点删除算法
    if (event->key() == Qt::Key_A)
    {
        m_record_poly_clip = !m_record_poly_clip;
        if (!m_record_poly_clip)
        {
            //多边形框进行删除操作
            // 点击多个点生成多边形，对多变形区域内的面片进行删除
            qDebug() << "before box choose  , mesh size " << this->mesh.polygons.size();
            vtkNew<vtkPoints> selectionPoints;
            GLdouble wx, wy, wz;
            for (int i = 0; i < m_points.size(); i++)
            {
                convScreen2World(m_points[i], wx, wy, wz);
                selectionPoints->InsertPoint(i, wx, wy, wz);
            }
            vtkSmartPointer<vtkPolyData> polydata3 = vtkSmartPointer<vtkPolyData>::New();
            pcl::io::mesh2vtk(this->mesh, polydata3);
            vtkSmartPointer<vtkSelectPolyData> selectPolyData = vtkSmartPointer<vtkSelectPolyData>::New();
            selectPolyData->SetInputData(polydata3);
            selectPolyData->SetLoop(selectionPoints);
            selectPolyData->GenerateUnselectedOutputOn();
            selectPolyData->Update();
            pcl::io::vtk2mesh(selectPolyData->GetUnselectedOutput(), mesh);
            qDebug() << "after box choose  , mesh size " << this->mesh.polygons.size();
            glDataProc->getRenderData(mesh);
            setImageData(glDataProc->glMeshData);
            repaint();
            m_points.clear();
        }

    }

}
void MyGLWidget::keyReleaseEvent(QKeyEvent* event) {
    if (event->key() & Qt::Key_Shift) 
        isShiftPressed = false;
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