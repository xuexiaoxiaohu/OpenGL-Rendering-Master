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
    void setCameraPara(QVector3D pos, QVector3D front);

protected:
    void initializeGL()         override;
    void paintGL()              override;
    void resizeGL(int w, int h) override; 
    void initializeShader();
    void mousePressEvent(QMouseEvent* event);
    void mouseReleaseEvent(QMouseEvent* event);
    void mouseMoveEvent(QMouseEvent* event);
    void wheelEvent(QWheelEvent* event);
    void keyPressEvent(QKeyEvent* event);
    void keyReleaseEvent(QKeyEvent* event);
    void setPressPosition(QPoint p_ab);
    void translate_point(QPoint& p_ab);

private:
    ShaderProgram* meshShader;
    ShaderProgram* pointShader;

    QMatrix4x4 model;
    QMatrix4x4 modelUse;
    QMatrix4x4 modelSave;
    Camera* camera;
    QMatrix4x4 proj;

    QOpenGLFunctions_4_5_Core* glFunc;
    std::vector<GLfloat> vertices;
    int dataType;
    QPoint pressPosition; 
    GLuint pointVAO, pointVBO;
    GLuint meshVAO, meshVBO;
};

#endif 