#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H
//Qt
#include <QOpenGLWidget>
#include <QKeyEvent>
#include <QOpenGLExtraFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
//Custom
#include <DataProcessing.h>
#include "Camera.h"
enum DataType {
    PointType,
    MeshType,
};

class MyGLWidget:public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core {
    Q_OBJECT
public:
    MyGLWidget(QWidget* parent, int DT);
    ~MyGLWidget();

    void setImageData(std::vector<GLfloat> data);
    void setCameraPara(QVector3D eye, QVector3D dir);
    void setMeshVertices(std::vector<QVector3D> meshVertexs);
    void setMesh(pcl::PolygonMesh mesh);

    pcl::PolygonMesh mesh;
    std::vector<QVector3D> allVertices;
    DataProcessing* glDataProc;
    bool isConstructionFinished = false;

protected:
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override; 

    void mousePressEvent(QMouseEvent* event)    override;
    void mouseReleaseEvent(QMouseEvent* event)  override;
    void mouseMoveEvent(QMouseEvent* event)     override;
    void wheelEvent(QWheelEvent* event)         override;
    void keyPressEvent(QKeyEvent* event)        override;
    void keyReleaseEvent(QKeyEvent* event)      override;

    void initializeShader();

    QVector3D convScreen2World(QPoint point);
    void rotateMesh(float angle, QVector3D axis);

private:
    QOpenGLShaderProgram* pShaderProgram;
    QOpenGLShaderProgram* mShaderProgram;
    bool isShiftPressed = false;

    QMatrix4x4 model;
    Camera* camera;
    QMatrix4x4 proj;

    std::vector<GLfloat> vertices;
    int dataType;
    QPoint pressPosition; 
    GLuint pVAO, pVBO;
    GLuint mVAO, mVBO;

    float rotationAngle = 0.0f;
    QVector3D rotationAxis = QVector3D{ 0.0f, 1.0f, 0.0f };
    QPoint m_lastPos;
    QPoint mMousePos;
};

#endif 