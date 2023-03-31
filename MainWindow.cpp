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

            QVector3D cameraPos = center + QVector3D(0.0f, 0.0f, size.z() * 2.0f);
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
                        std::string curAppPath = meshDataProc->getAppPath();
                        std::string oriPlyPath = curAppPath + "/result.ply";
                        pcl::PolygonMesh inMesh;
                        pcl::PolygonMesh outMesh;
                        pcl::io::loadPLYFile(oriPlyPath, inMesh);
                        meshDataProc->addNormalForMesh(inMesh, outMesh);
                        std::string aaa = curAppPath + "/aaa.ply";
                        pcl::io::savePLYFile(aaa, outMesh);
               
            /*            int nr_points = outMesh.cloud.width * outMesh.cloud.height;
                        int nr_faces = outMesh.polygons.size();
                        if (nr_points == 0) return;
                        int point_size = outMesh.cloud.data.size() / nr_points;
                        meshData.clear();

                        for (std::size_t i = 0; i < nr_faces; i++) {
                            for (std::size_t j = 0; j < outMesh.polygons[i].vertices.size(); j++)
                                std::cout << outMesh.polygons[i].vertices[j] << " ";
                        }


                        for (std::size_t i = 0; i < nr_points; i++) {
                            for (std::size_t d = 0; d < outMesh.cloud.fields.size(); ++d) {
                                if ((outMesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
                                    outMesh.cloud.fields[d].name == "x" ||
                                    outMesh.cloud.fields[d].name == "y" ||
                                    outMesh.cloud.fields[d].name == "z" ||
                                    outMesh.cloud.fields[d].name == "normal_x" ||
                                    outMesh.cloud.fields[d].name == "normal_y" ||
                                    outMesh.cloud.fields[d].name == "normal_z"
                                    )) {
                                    float value;
                                    memcpy(&value, &outMesh.cloud.data[i * point_size + outMesh.cloud.fields[d].offset], sizeof(float));
                                    meshData.emplace_back(value);
                                }
                            }
                        }
                        myMeshGLWidget->setImageData(meshData);
                        Sleep(1000);*/

                        std::string oriPcdPath = curAppPath + "/result.pcd";
                        std::string finalMeshPath = curAppPath + "/finalMesh.ply";
                        myMeshGLWidget->setCameraPara(cameraPos, cameraTarget);
                        meshDataProc->ply2pcd(oriPlyPath, oriPcdPath);
                        meshDataProc->getNormalVector(oriPcdPath);
      
                        meshDataProc->writePlyData(inMesh);
                        meshDataProc->loadMeshData(finalMeshPath.data());

                        for (int i = 0, meshLineMarker = 0; i < meshDataProc->surfaceModelData.vecFaceTriangles.size() / 3; i++) {
                            if (i == 0) meshData.clear();
                            meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker]);
                            meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker + 1]);
                            meshData.emplace_back(meshDataProc->surfaceModelData.vecFaceTriangles[meshLineMarker + 2]);

                            meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker]);
                            meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker + 1]);
                            meshData.emplace_back(meshDataProc->surfaceModelData.vecVertexNormals[meshLineMarker + 2]);

                            meshLineMarker += 3;
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