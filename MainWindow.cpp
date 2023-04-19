#include "MainWindow.h"
#include <QFileDialog>
#include "MyGLWidget.h"
#include "Macro.h"
#include <QPixmap>
#include <QPainter>
MainWindow::MainWindow(QWidget *parent):
    QMainWindow(parent){
	ui.setupUi(this);
    addOpengGLWidget();
    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(callbackRepaint()));
    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
    timer->start(10);
    pointProc = new DataProcessing();
    meshProc = new DataProcessing();
    surface = new SurfaceReconsturction();
}
void MainWindow::updateCursor() {
    if (mMeshGLWidget->geometry().contains(this->mapFromGlobal(QCursor::pos()))) {
        mMeshGLWidget->isMouseBrush = true;
    }else{
        mMeshGLWidget->isMouseBrush = false;
    }
}
MainWindow::~MainWindow(){
    delete mPointGLWidget;
    delete mMeshGLWidget;
    delete surface;
    delete pointProc;
    delete meshProc;
}

void MainWindow::addOpengGLWidget(){
    mPointGLWidget = new MyGLWidget(this, PointType);
    mPointGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(mPointGLWidget);
    mMeshGLWidget = new MyGLWidget(this, MeshType);
    mMeshGLWidget->setFixedSize(SCR_WIDTH, SCR_HEIGHT);
    ui.openGLHorizontalLayout->addWidget(mMeshGLWidget);
}
void MainWindow::callbackRepaint() {
    if (ui.pointCheckBox->checkState() == Qt::Checked) {
        mPointGLWidget->repaint();
    }
    if (ui.meshCheckBox->checkState() == Qt::Checked) {
        mMeshGLWidget->repaint();
    }
}
void MainWindow::openFile(){
    QString fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", 
        "./Release/Data", "Scan Data(*.txt)");
    if (fileName.isEmpty()) return;

    ui.lineEdit_file->setText(fileName);
    pointProc->loadPointData(fileName.toStdString().c_str());
}
void MainWindow::startRendering(){
    auto collectDataFunc = [=]() {
        std::vector<QVector3D> rawData;
        for (int i = 0; i < pointProc->pointData.size(); i++){
            rawData.emplace_back(QVector3D(pointProc->pointData[i].x(), 
            pointProc->pointData[i].y(), pointProc->pointData[i].z()));
            pointProc->getMaxMinPoint(rawData);

            QVector3D center = (pointProc->maxPoint + pointProc->minPoint) / 2.0f;
            float radius = (pointProc->maxPoint - center).length();

            if (ui.pointCheckBox->checkState() == Qt::Checked) {
                std::vector<GLfloat> glPoint;
                for (int i = 0; i < rawData.size(); i++) {
                    glPoint.emplace_back(pointProc->pointData[i].x());
                    glPoint.emplace_back(pointProc->pointData[i].y());
                    glPoint.emplace_back(pointProc->pointData[i].z());
                }
                mPointGLWidget->setAdaptivePara(center, radius);
                mPointGLWidget->setImageData(glPoint);
            }
            if (ui.meshCheckBox->checkState() == Qt::Checked) {
                if ((rawData.size() >= MIN_PTS_SIZE_REQD)) {
                    int diff = static_cast<int>(rawData.size()) - static_cast<int>(pointProc->pointData.size());
                    if (((rawData.size() % MESH_GRTH_SIZE) == 0) || (abs(diff) <= 0)) {
                        surface->construction(rawData);
                

                        std::string curAppPath = meshProc->getAppPath();
                        std::string oriPath = "C:/Project/OpenGL-Rendering-Master-Build/result.ply";
                        std::string dstPath = "C:/Project/OpenGL-Rendering-Master-Build/triangleResult.ply";
                        pcl::PolygonMesh mesh;

                        meshProc->poly2tri(oriPath, dstPath);
                        pcl::io::loadPLYFile(dstPath, mesh);
                        meshProc->addNV(mesh);
              
                        pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
                        pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);
  
                        std::vector<GLfloat> glMesh;
                        std::vector<QVector3D> glVtx;
                        for (std::size_t i = 0; i < mesh.polygons.size(); i++) {
                            for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
                                pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
                                glMesh.emplace_back(point.x);
                                glMesh.emplace_back(point.y);
                                glMesh.emplace_back(point.z);
                                glMesh.emplace_back(point.normal_x);
                                glMesh.emplace_back(point.normal_y);
                                glMesh.emplace_back(point.normal_z);

                                glVtx.emplace_back(point.x, point.y, point.z);
                            }
                        }
                        if ((abs(diff) <= 0)){
                            mMeshGLWidget->isConstrFin = true;
                            mMeshGLWidget->setMesh(mesh);
                            mMeshGLWidget->setMeshVtx(glVtx);
                        }
   
                        mMeshGLWidget->setAdaptivePara(center, radius);
                        mMeshGLWidget->setImageData(glMesh);
                    }
                }
            }
            emit signal_glUpdate();
        }
    };
    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}
