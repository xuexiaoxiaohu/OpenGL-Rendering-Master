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

enum DataType 
{
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
    void setMesh(pcl::PolygonMesh mesh);

    pcl::PolygonMesh mesh;
    DataProcessing* glDataProc;
    bool isMouseBrush;
    float grayValue;

public slots:
    void updateCursor();

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

private:
    QOpenGLShaderProgram* pShader, * mShader;

    QMatrix4x4 model, proj;
    Camera* camera;

    std::vector<GLfloat> vertices;
    int dataType;
    GLuint pVAO, pVBO, mVAO, mVBO;

    bool isShiftPressed;
    float brushParam, brushSize, rotationAngle;
    QPoint pressPosition, lastMousePos;
    GLdouble lastWx = 0, lastWy = 0, lastWz = 0;
    QVector3D brushPosition;
};

#endif 