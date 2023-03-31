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

            myPointGLWidget->setCameraPara(cameraPos, cameraTarget);

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                pointDataProc->centralizeOriginalPoints(originalPointData);

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

                        /*
                        * VTK 
                        string aaa = meshDataProc->getAppPath();
                        std::string oriPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                        std::string transMeshPlyPath = "C:/Project/OpenGL-Rendering-Master-Build/transMesh.ply";
                        std::string transMeshPcdPath = "C:/Project/OpenGL-Rendering-Master-Build/transMesh.pcd";
                        std::string finalMeshPath = "C:/Project/OpenGL-Rendering-Master-Build/finalMesh.ply";

                        meshDataProc->ply2ply(oriPlyPath, transMeshPlyPath);
                        meshDataProc->ply2pcd(transMeshPlyPath, transMeshPcdPath);
                        meshDataProc->getNormalVector(transMeshPcdPath);
                        pcl::PolygonMesh mesh;
                        pcl::io::loadPLYFile(transMeshPlyPath, mesh);
                        meshDataProc->getMeshData(mesh);
                        meshDataProc->writePlyData(mesh);
                        meshDataProc->loadMeshData(finalMeshPath.data());
                        */
                        // Open3D
                        std::string curAppPath = meshDataProc->getAppPath();
                        std::string oriPlyPath = curAppPath + "/result.ply";
                        std::string oriPcdPath = curAppPath + "/result.pcd";
                        std::string finalMeshPath = curAppPath + "/finalMesh.ply";

                        meshDataProc->ply2pcd(oriPlyPath, oriPcdPath);
                        meshDataProc->getNormalVector(oriPcdPath);
                        pcl::PolygonMesh mesh;
                        pcl::io::loadPLYFile(oriPlyPath, mesh);
                        meshDataProc->writePlyData(mesh);
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