#include "MainWindow.h"
#include <QFileDialog>
#include <QElapsedtimer>
#include <qelapsedtimer.h>
#include <QDebug>
#include "MyGLWidget.h"
#include "Macro.h"

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

            QVector3D center = (pointDataProc->maxCoord + pointDataProc->minCoord) / 2.0f;
            QVector3D size = pointDataProc->maxCoord - pointDataProc->minCoord;

            QVector3D cameraPos = center + QVector3D(0.0f, 0.0f, size.z() * 3.0f);
            QVector3D cameraTarget = center;

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                myPointGLWidget->setCameraPara(cameraPos, cameraTarget);
                this->glPointData.resize(3 * rawData.size());
                for (int i = 0, pointLineMarker = 0; i < rawData.size(); i++) {
                    glPointData[pointLineMarker++] = pointDataProc->pointData[i].x();
                    glPointData[pointLineMarker++] = pointDataProc->pointData[i].y();
                    glPointData[pointLineMarker++] = pointDataProc->pointData[i].z();
                }
                myPointGLWidget->setImageData(glPointData);
            }
            if (ui.meshCheckBox->checkState() == Qt::Checked) {
                if ((rawData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                    if (((pointLine % MESH_INCREASE_SIZE) == 0) || (pointLine >= pointDataProc->pointData.size())) {
                        surface->construction(rawData);
                        myMeshGLWidget->setCameraPara(cameraPos, cameraTarget);
                        std::string curAppPath = meshDataProc->getAppPath();
                        std::string oriPlyPath = curAppPath + "/result.ply";
                        pcl::PolygonMesh inMesh, outMesh;
                        pcl::io::loadPLYFile(oriPlyPath, inMesh);
                        meshDataProc->addNormalForMesh(inMesh, outMesh);
              
                        pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
                        pcl::fromPCLPointCloud2(outMesh.cloud, *pointsPtr);

                        for (std::size_t i = 0; i < outMesh.polygons.size(); i++) {
                            if (i == 0)  glMeshData.clear();
                            for (std::size_t j = 0; j < outMesh.polygons[i].vertices.size(); j++) {
                                pcl::PointNormal point = pointsPtr->points[outMesh.polygons[i].vertices[j]];
                                glMeshData.emplace_back(point.x);
                                glMeshData.emplace_back(point.y);
                                glMeshData.emplace_back(point.z);
                                glMeshData.emplace_back(point.normal_x);
                                glMeshData.emplace_back(point.normal_y);
                                glMeshData.emplace_back(point.normal_z);
                            }
                        }
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
void MainWindow::startUpdateGL() {
    if (ui.pointCheckBox->checkState() == Qt::Checked) {
        myPointGLWidget->repaint();
    }
    if (ui.meshCheckBox->checkState() == Qt::Checked) {
        myMeshGLWidget->repaint();
    }

}