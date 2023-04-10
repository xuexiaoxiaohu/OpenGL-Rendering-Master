#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H
#include <QOpenGLWidget>
#include <QtMath>
#include <QKeyEvent>
#include "ShaderProgram.h"
#include "Camera.h"
#include <QOpenGLExtraFunctions>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QOpenGLFunctions_4_5_Core>
#include <DataProcessing.h>

enum DataType 
{
    PointType,
    MeshType,
};

class MyGLWidget:public QOpenGLWidget{
    Q_OBJECT
public:
    MyGLWidget(QWidget* parent, int DT);
    ~MyGLWidget();
    void setImageData(std::vector<GLfloat> data);
    void setCameraPara(QVector3D eye, QVector3D center);
    void setMeshVertices(std::vector<QVector3D> meshVertexs);
    void setMesh(pcl::PolygonMesh mesh);

    pcl::PolygonMesh mesh;
    std::vector<QVector3D> allVertices;
    bool isConstructionFinished = false;

protected:
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override; 

    void mousePressEvent(QMouseEvent* event)  override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

    void initializeShader();

    QVector3D convertScreenToWorld(QPoint point);
    void rotateMesh(float angle, QVector3D axis);
    DataProcessing* dataProc;

private:
    ShaderProgram* meshShader;
    ShaderProgram* pointShader;
    bool isShiftPressed = false;

    QMatrix4x4 modelMatrix;
    Camera* camera;
    QMatrix4x4 projMatrix;

    QOpenGLFunctions_4_5_Core* glFunc;
    std::vector<GLfloat> vertices;
    int dataType;
    QPoint pressPosition; 
    GLuint pointVAO, pointVBO;
    GLuint meshVAO, meshVBO;

    float rotationAngle = 0.0f;
    QVector3D rotationAxis = QVector3D{ 0.0f, 1.0f, 0.0f };
    QPoint m_lastPos;
    QPoint mMousePos;
};

#endif 