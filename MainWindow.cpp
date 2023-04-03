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
    isOpenGLThreadStart = true;
    addOpengGLWidget();
    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(chooseFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
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
    QString fileName = QFileDialog::getOpenFileName(nullptr, "open", ".", "*.txt");
    if (!fileName.isEmpty()){
        ui.lineEdit_file->setText(fileName);
        pointDataProc->loadPointData(fileName.toStdString().c_str());
        pointData3D.resize(pointDataProc->pointData.size());
        pointData3D = pointDataProc->pointData;
    }
}
// Begin render
void MainWindow::startRendering(){
    auto collectDataFunc = [=]() {
        for (int pointLine = 0; pointLine < pointData3D.size(); pointLine++){
            originalPointData.emplace_back(QVector3D{pointData3D[pointLine].x(), pointData3D[pointLine].y(), pointData3D[pointLine].z()});
            pointDataProc->getMaxMinCoord(originalPointData);

            QVector3D center = (pointDataProc->maxCoord + pointDataProc->minCoord) / 2.0f;
            QVector3D size = pointDataProc->maxCoord - pointDataProc->minCoord;

            QVector3D cameraPos = center + QVector3D(0.0f, 0.0f, size.z() * 3.0f);
            QVector3D cameraTarget = center;

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                myPointGLWidget->setCameraPara(cameraPos, cameraTarget);
                pointData.resize(3 * originalPointData.size());
                for (int i = 0, pointLineMarker = 0; i < originalPointData.size(); i++) {
                    pointData[pointLineMarker++] = pointDataProc->pointData[i].x();
                    pointData[pointLineMarker++] = pointDataProc->pointData[i].y();
                    pointData[pointLineMarker++] = pointDataProc->pointData[i].z();
                }
                myPointGLWidget->setImageData(pointData);
            }
            if (ui.meshCheckBox->checkState() == Qt::Checked) {
               
                if ((originalPointData.size() >= MIN_POINTS_SIZE_REQUIRED)) {
                    if (((pointLine % MESH_INCREASE_SIZE) == 0) || (pointLine >= pointData3D.size())) {
                        surface->construction(originalPointData);
                        myMeshGLWidget->setCameraPara(cameraPos, cameraTarget);
                        std::string curAppPath = meshDataProc->getAppPath();
                        std::string oriPlyPath = curAppPath + "/result.ply";
                        pcl::PolygonMesh inMesh;
                        pcl::PolygonMesh outMesh;
                        pcl::io::loadPLYFile(oriPlyPath, inMesh);
                        meshDataProc->addNormalForMesh(inMesh, outMesh);
               
                        int nr_points = outMesh.cloud.width * outMesh.cloud.height;
                        int nr_faces = outMesh.polygons.size();
                        int point_size = outMesh.cloud.data.size() / nr_points;
                        meshData.clear();
                        pcl::PointCloud<pcl::PointNormal>::Ptr cloud111(new pcl::PointCloud<pcl::PointNormal>);
                        pcl::fromPCLPointCloud2(outMesh.cloud, *cloud111);
                        for (std::size_t i = 0; i < nr_faces; i++) {
                            for (std::size_t j = 0; j < outMesh.polygons[i].vertices.size(); j++) {
                                const pcl::Vertices& vertices = outMesh.polygons[outMesh.polygons[i].vertices[j]];
                                for (size_t j = 0; j < vertices.vertices.size(); j++){
                                    int index = vertices.vertices[j];
                                    pcl::PointNormal point = cloud111->points[index];
                                    meshData.emplace_back(point.x);
                                    meshData.emplace_back(point.y);
                                    meshData.emplace_back(point.z);
                                    meshData.emplace_back(point.normal_x);
                                    meshData.emplace_back(point.normal_y);
                                    meshData.emplace_back(point.normal_z);
                                    
                                }
                            }
                        }
                        myMeshGLWidget->setImageData(meshData);
                    }
                }
            }
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}