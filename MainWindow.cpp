#include "MainWindow.h"
#include <QFileDialog>
#include <QElapsedtimer>
#include <qelapsedtimer.h>
#include <QDebug>
#include "MyGLWidget.h"
#include "Macro.h"
#include <stdlib.h>
MainWindow::MainWindow(QWidget *parent): QMainWindow(parent){
	ui.setupUi(this);
    addOpengGLWidget();
    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(callbackRepaint()));

    pointProc = new DataProcessing();
    meshProc = new DataProcessing();
    surface = new SurfaceReconsturction();
}
MainWindow::~MainWindow(){
    delete myPointGLWidget;
    delete myMeshGLWidget;
    delete pointProc;
    delete meshProc;
    delete surface;
}

void MainWindow::addOpengGLWidget(){
    myPointGLWidget = new MyGLWidget(this, PointType);
    myPointGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myPointGLWidget);
    myMeshGLWidget = new MyGLWidget(this, MeshType);
    myMeshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myMeshGLWidget);
}
void MainWindow::callbackRepaint() {
    if (ui.pointCheckBox->checkState() == Qt::Checked) {
        myPointGLWidget->repaint();
    }
    if (ui.meshCheckBox->checkState() == Qt::Checked) {
        myMeshGLWidget->repaint();
    }
}
void MainWindow::openFile(){
    QString fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", "./Release/Data", "Scan Data(*.txt)");
    if (fileName.isEmpty()) return;
    ui.lineEdit_file->setText(fileName);
    pointProc->loadPointData(fileName.toStdString().c_str());
}
void MainWindow::startRendering(){
    auto collectDataFunc = [=]() {
        for (int i = 0; i < pointProc->pointData.size(); i++){
            rawData.emplace_back(QVector3D(pointProc->pointData[i].x(), 
            pointProc->pointData[i].y(), pointProc->pointData[i].z()));
            pointProc->getMaxMinPoint(rawData);

            QVector3D center = (pointProc->maxPoint + pointProc->minPoint)/ 2.0f;
            float radius = (pointProc->maxPoint - center).length();

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                for (int i = 0; i < rawData.size(); i++) {
                    if (i == 0) glPoint.clear();
                    glPoint.emplace_back(pointProc->pointData[i].x());
                    glPoint.emplace_back(pointProc->pointData[i].y());
                    glPoint.emplace_back(pointProc->pointData[i].z());
                }
                myPointGLWidget->setCameraPara(center, radius);
                myPointGLWidget->setImageData(glPoint);
            }
            if (ui.meshCheckBox->checkState() == Qt::Checked) {
                if ((rawData.size() >= MIN_PTS_SIZE_REQD)) {
                    int diff = (int)rawData.size() - (int)pointProc->pointData.size();
                    if (((rawData.size() % MESH_GRTH_SIZE) == 0) || (abs(diff) <= 0)) {
                        surface->construction(rawData);
                      
                        std::string curAppPath = meshProc->getAppPath();
                        std::string oriPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                        std::string dstPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/triangleResult.ply";
                        pcl::PolygonMesh mesh;

                        meshProc->poly2tri(oriPlyPath, dstPlyPath);
                        pcl::io::loadPLYFile(dstPlyPath, mesh);
                        meshProc->addNormalVector(mesh);
              
                        pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
                        pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);

                        for (std::size_t i = 0; i < mesh.polygons.size(); i++) {
                            if (i == 0) {
                                glMesh.clear();
                                glMeshVertices.clear();
                            }
                            for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
                                pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
                                glMesh.emplace_back(point.x);
                                glMesh.emplace_back(point.y);
                                glMesh.emplace_back(point.z);
                                glMesh.emplace_back(point.normal_x);
                                glMesh.emplace_back(point.normal_y);
                                glMesh.emplace_back(point.normal_z);

                                glMeshVertices.emplace_back(QVector3D{ point.x, point.y, point.z });
                            }
                        }
                        if ((abs(diff) <= 0)){
                            myMeshGLWidget->isConstructionFinished = true;
                            myMeshGLWidget->setMesh(mesh);
                            myMeshGLWidget->setMeshVertices(glMeshVertices);
                        }
   
                        myMeshGLWidget->setCameraPara(center, radius);
                        myMeshGLWidget->setImageData(glMesh);
                    }
                }
            }
            emit signal_glUpdate();
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
