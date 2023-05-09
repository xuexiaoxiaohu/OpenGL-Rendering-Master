#include "MainWindow.h"
#include <filesystem>
#include <QFileDialog>
#include "MyGLWidget.h"
#include "Macro.h"
#include <QPixmap>
#include <QPainter>
MainWindow::MainWindow(QWidget *parent):QMainWindow(parent),isRenderRunning(true){
	ui.setupUi(this);
    addOpengGLWidget();

    connect(ui.openPushBtn, SIGNAL(clicked()), this, SLOT(openFile()));
    connect(ui.startPushBtn, SIGNAL(clicked()), this, SLOT(startRendering()));
    connect(ui.stopPushBtn, SIGNAL(clicked()), this, SLOT(stopRendering()));
    connect(this, SIGNAL(signal_glUpdate()), this, SLOT(RepaintUI()));

    driver = new NDIDriver("COM3");
    surface = new SurfaceReconsturction();
    pointProc = new DataProcessing();
    meshProc = new DataProcessing();

    QTimer* timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(updateCursor()));
    timer->start(100);
}
void MainWindow::updateCursor() {
    mMeshGLWidget->geometry().contains(this->mapFromGlobal(QCursor::pos())) ? 
        mMeshGLWidget->isMouseBrush = true:false;
}
MainWindow::~MainWindow(){
    delete driver;
    delete surface;
    delete mPointGLWidget;
    delete mMeshGLWidget;
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
void MainWindow::RepaintUI() {
    mPointGLWidget->repaint();
    mMeshGLWidget->repaint();
}
void MainWindow::openFile(){
    QString fileName = QFileDialog::getOpenFileName(this, "Open Scan Data", "./Release/Data", "Scan Data(*.txt)");
    if (fileName.isEmpty()) return;

    ui.fileLineEdit->setText(fileName);
    pointProc->loadPointData(fileName);
}
void MainWindow::startRendering(){
    auto collectDataFunc = [=]() {
        // Get data From NDI device.
        if (ui.fileLineEdit->text().contains("txt") == false) {
            while (!driver->set_tracking_status(true)) Sleep(10);
       
            while (isRenderRunning) {
                static float lastTx = 0, lastTy = 0, lastTz = 0;
                if (auto data = driver->get_data(); data.has_value()) {
                    auto& items = data.value();
                    for (auto& item : items) {
                        if (!item.transform.isMissing()) {
                            auto point = item.transform;
                            if (point.toolHandle == 11) {
                                if ((abs(lastTx - point.tx) > DELTA) && (abs(lastTy - point.ty) > DELTA)
                                    && (abs(lastTz - point.tz) > DELTA)) {
                                    if ((abs(point.tx) < VOLUME_MAX) && (abs(point.tz) < VOLUME_MAX)) {
                                        pointProc->pointData.push_back(QVector3D{ (float)point.tx ,(float)point.ty ,(float)point.tz });
                                        lastTx = point.tx; lastTy = point.ty; lastTz = point.tz;
                                    }
                                }
                            }
                            std::cout << item.timespec_s << " ," << item.frameNumber << " , " << point.toolHandle << ",  "
                                << point.status << " , " << point.q0 << " , " << point.qx << " , " << point.qy << " , "
                                << point.qz << ",  " << point.tx << ",  " << point.ty << ",  " << point.tz << std::endl;
                        }
                    }
                }
                enclosureDataProcessing();
            }
        }else{
            enclosureDataProcessing();
        }
    };

    std::thread collectDataThread(collectDataFunc);
    collectDataThread.detach();
}

void MainWindow::stopRendering() {
    driver->set_tracking_status(false);
    isRenderRunning = false;

}
void MainWindow::enclosureDataProcessing(){
    std::vector<QVector3D> rawData;
    for (int i = 0; i < pointProc->pointData.size(); i++) {
        rawData.emplace_back(pointProc->pointData[i]);
        pointProc->getMaxMinPoint(rawData);

        QVector3D center = (pointProc->maxPoint + pointProc->minPoint) / 2.0f;
        float radius = (pointProc->maxPoint - center).length();

        // Point
        std::vector<GLfloat> glPoint;
        for (int i = 0; i < rawData.size(); i++) {
            glPoint.emplace_back(pointProc->pointData[i].x());
            glPoint.emplace_back(pointProc->pointData[i].y());
            glPoint.emplace_back(pointProc->pointData[i].z());
        }
        mPointGLWidget->setAdaptivePara(center, radius);
        mPointGLWidget->setImageData(glPoint);

        // Mesh 
        if ((rawData.size() >= MIN_PTS_SIZE_REQD)) {
            int diff = static_cast<int>(rawData.size()) - static_cast<int>(pointProc->pointData.size());
            if (((rawData.size() % MESH_GRTH_SIZE) == 0) || (abs(diff) <= 0)) {
                surface->construction(rawData);

                std::filesystem::path parentPath = std::filesystem::current_path();
                std::string srcPath = parentPath.string() + "/result.ply";
                std::string dstPath = parentPath.string() + "/triangleResult.ply";
                pcl::PolygonMesh mesh;
                meshProc->isoExpRemeshing(srcPath.c_str(), dstPath.c_str());

                pcl::io::loadPLYFile(dstPath.c_str(), mesh);
                meshProc->addNormal(mesh);

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
                mMeshGLWidget->setMesh(mesh);
                mMeshGLWidget->setMeshVtx(glVtx);
                mMeshGLWidget->setAdaptivePara(center, radius);
                mMeshGLWidget->setImageData(glMesh);
            }
        }
        emit signal_glUpdate();
    }
}