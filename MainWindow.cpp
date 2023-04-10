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
    pointDataProc     = new DataProcessing();
    meshDataProc      = new DataProcessing();
    surface           = new SurfaceReconsturction();
    addOpengGLWidget();
    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(chooseFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(startUpdateGL()));
}
MainWindow::~MainWindow(){
    delete surface;
    delete myPointGLWidget;
    delete myMeshGLWidget;
    delete pointDataProc;
    delete meshDataProc;
}

// add opengl widget
void MainWindow::addOpengGLWidget(){
    myPointGLWidget = new MyGLWidget(this, PointType);
    myPointGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myPointGLWidget);
    myMeshGLWidget = new MyGLWidget(this, MeshType);
    myMeshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(myMeshGLWidget);
}
void MainWindow::startUpdateGL() {
    if (ui.pointCheckBox->checkState() == Qt::Checked) {
        myPointGLWidget->repaint();
    }
    if (ui.meshCheckBox->checkState() == Qt::Checked) {
        myMeshGLWidget->repaint();
    }
}
// Choose File
void MainWindow::chooseFile(){
    QString fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", "./Release/Data", "Scan Data(*.txt)");
    
    if (fileName.isEmpty()) return;

    ui.lineEdit_file->setText(fileName);
    pointDataProc->loadPointData(fileName.toStdString().c_str());
}
// Begin render
void MainWindow::startRendering(){
    auto collectDataFunc = [=]() {
        for (int pointLine = 0; pointLine < pointDataProc->pointData.size(); pointLine++){
            rawData.emplace_back(QVector3D{ pointDataProc->pointData[pointLine].x(), pointDataProc->pointData[pointLine].y(), pointDataProc->pointData[pointLine].z()});
            pointDataProc->getMaxMinCoord(rawData);

            QVector3D cameraDir = (pointDataProc->maxCoord + pointDataProc->minCoord) / 2.0f;
            QVector3D cameraEye = cameraDir + QVector3D(0.0f, 0.0f, (pointDataProc->maxCoord - pointDataProc->minCoord).z() * 2.0f);

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                for (int i = 0; i < rawData.size(); i++) {
                    if (i == 0) glPointData.clear();
                    glPointData.emplace_back(pointDataProc->pointData[i].x());
                    glPointData.emplace_back(pointDataProc->pointData[i].y());
                    glPointData.emplace_back(pointDataProc->pointData[i].z());
                }
                myPointGLWidget->setCameraPara(cameraEye, cameraDir);
                myPointGLWidget->setImageData(glPointData);
            }
            if (ui.meshCheckBox->checkState() == Qt::Checked) {
                if ((rawData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                    int diff = static_cast<int>(rawData.size()) - static_cast<int>(pointDataProc->pointData.size());
                    if ((((rawData.size()) % MESH_INCREASE_SIZE) == 0) || (abs(diff) <= 0)) {
                        surface->construction(rawData);
                      
                        std::string curAppPath = meshDataProc->getAppPath();
                        std::string oriPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                        std::string dstPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/triangleResult.ply";
                        meshDataProc->ply2ply(oriPlyPath, dstPlyPath);
                        pcl::PolygonMesh inMesh, outMesh;
                        pcl::io::loadPLYFile(dstPlyPath, inMesh);
                        meshDataProc->addNormalForMesh(inMesh, outMesh);
              
                        pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
                        pcl::fromPCLPointCloud2(outMesh.cloud, *pointsPtr);

                        for (std::size_t i = 0; i < outMesh.polygons.size(); i++) {
                            if (i == 0) {
                                glMeshData.clear();
                                allVertices.clear();
                            }
                            for (std::size_t j = 0; j < outMesh.polygons[i].vertices.size(); j++) {
                                pcl::PointNormal point = pointsPtr->points[outMesh.polygons[i].vertices[j]];
                                glMeshData.emplace_back(point.x);
                                glMeshData.emplace_back(point.y);
                                glMeshData.emplace_back(point.z);
                                glMeshData.emplace_back(point.normal_x);
                                glMeshData.emplace_back(point.normal_y);
                                glMeshData.emplace_back(point.normal_z);

                                allVertices.emplace_back(QVector3D{ point.y ,point.y,point.z });
                            }
                        }
                        if ((abs(diff) <= 0)){
                            meshDataProc->isConstructionFinished = true;
                            meshDataProc->setMesh(outMesh);
                            meshDataProc->setMeshVertices(allVertices);
                        }
   
                        myMeshGLWidget->setCameraPara(cameraEye, cameraDir);
                        myMeshGLWidget->setImageData(glMeshData);
                    }
                }
            }
            emit signal_glUpdate();
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
