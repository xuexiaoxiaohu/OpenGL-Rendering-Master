#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"

DataProcessing::DataProcessing() {

}

DataProcessing::~DataProcessing() {

}

std::string DataProcessing::getAppPath() {
	QString qAppDir = QCoreApplication::applicationDirPath();
	std::string::size_type iPos = (qAppDir.toStdString().find_last_of('\\') + 1) == 0 ?qAppDir.toStdString().find_last_of('/') + 1 : qAppDir.toStdString().find_last_of('\\') + 1;
	return qAppDir.toStdString().substr(0, iPos);
}
void DataProcessing::loadPointData(const char* path) {
	std::fstream fs(path);
	if (fs.is_open() == NULL) return;
	float x, y, z;
	while (fs >> x >> y >> z) pointData.emplace_back(QVector3D {x, y, z});
	fs.close();
}

void DataProcessing::getMaxMinCoord(std::vector<QVector3D> data) {
	maxCoord = minCoord = {data[0].x() ,data[0].y() ,data[0].z()};
	for (int i = 0; i < data.size(); i++) {
		if (maxCoord.x() < data[i].x())	maxCoord.setX(data[i].x());
		if (maxCoord.y() < data[i].y())	maxCoord.setY(data[i].y());
		if (maxCoord.z() < data[i].z())	maxCoord.setZ(data[i].z());

		if (minCoord.x() > data[i].x())	minCoord.setX(data[i].x());
		if (minCoord.y() > data[i].y())	minCoord.setY(data[i].y());
		if (minCoord.z() > data[i].z())	minCoord.setZ(data[i].z());
	}
}

void DataProcessing::addNormalForMesh(pcl::PolygonMesh &inMesh, pcl::PolygonMesh &outMesh) {
	outMesh = inMesh;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(inMesh.cloud, *cloud);

	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;
	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud);
	search.nearestKSearch(*cloud, pcl::Indices(), NUM_NEIGHBORS, k_indices, k_sqr_distances);

	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsRefinedPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	for (size_t i = 0; i < cloud->size(); ++i) {
		pcl::Normal normal;
		ne.computePointNormal(*cloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*cloud)[i], (*cloud).sensor_origin_[0], (*cloud).sensor_origin_[1],(*cloud).sensor_origin_[2], normal.normal_x, normal.normal_y, normal.normal_z);
		normals->emplace_back(normal);
	}

	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normals);
	nr.setMaxIterations(ITERATIONS);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefinedPtr);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normalsRefinedPtr, *cloud_with_normals);
	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*cloud_with_normals, outputCloud);
	outMesh.cloud = outputCloud;
}
