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
    void setAdaptivePara(QVector3D center, float radius);
    void setMeshVtx(std::vector<QVector3D> meshVertexs);
    void setMesh(pcl::PolygonMesh mesh);

    pcl::PolygonMesh mesh;
    std::vector<QVector3D> allVertices;
    DataProcessing* glDataProc;
    bool isConstrFin = false;

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

    void convScreen2World(QPoint point, GLdouble& wx, GLdouble& wy, GLdouble& wz);
    void rotateMesh(float angle, QVector3D axis);
    void translatePoint(QPoint& pressPos);
private:
    QOpenGLShaderProgram* pShaderProgram;
    QOpenGLShaderProgram* mShaderProgram;
    bool isShiftPressed;

    QMatrix4x4 model;
    Camera* camera;
    QMatrix4x4 proj;

    std::vector<GLfloat> vertices;
    int dataType;
    QPoint pressPosition; 
    GLuint pVAO, pVBO;
    GLuint mVAO, mVBO;

    float rotationAngle;
    QVector3D rotationAxis;
    QPoint m_lastPos;
    QPoint mMousePos;
};

#endif 