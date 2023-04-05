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
    void setCameraPara(QVector3D eye, QVector3D center);

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