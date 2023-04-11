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

int DataProcessing::findNearestVertex(QVector3D worldPos, std::vector<QVector3D> glMeshVertices) {
	int nearestVertexIndex = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < glMeshVertices.size(); i++) {
		float dist = (glMeshVertices[i] - worldPos).length();
		if (dist < minDist) {
			nearestVertexIndex = i;
			minDist = dist;
		}
	}
	return nearestVertexIndex;
}
std::vector<int> DataProcessing::findKNeighbors(pcl::PolygonMesh mesh, pcl::PointXYZ query_point) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);
	std::vector<int> k_indices;
	std::vector<float> k_distances;
	kdtree->nearestKSearch(query_point, 20, k_indices, k_distances);
	return k_indices;
}
pcl::PolygonMesh DataProcessing::eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete) {
	std::vector<pcl::Vertices>& polygons = mesh.polygons;
	for (int i = 0; i < polygons.size(); ++i) {
		pcl::Vertices& vertices = polygons[i];
		bool should_remove_polygon = false;
		for (int j = 0; j < verticesToDelete.size(); ++j) {
			int index = verticesToDelete[j];
			if (std::find(vertices.vertices.begin(), vertices.vertices.end(), index) != vertices.vertices.end()) {
				should_remove_polygon = true;
				break;
			}
		}
		if (should_remove_polygon) {
			polygons.erase(polygons.begin() + i);
			--i;
		}
	}
	mesh.polygons = polygons;
	return mesh;
}


void DataProcessing::getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, std::vector<QVector3D> allVertices){
	int index = findNearestVertex(worldPos, allVertices);
	if (index != -1) {
		pcl::PointXYZ nearestVertex;
		nearestVertex.x = allVertices[index].x();
		nearestVertex.y = allVertices[index].y();
		nearestVertex.z = allVertices[index].z();

		pcl::Indices toRemove = findKNeighbors(mesh, nearestVertex);
		eraseMesh(mesh, toRemove);
	}
}
void DataProcessing::getGLMeshData(pcl::PolygonMesh &mesh) {
	glMeshData.clear();
	pcl::PointCloud<pcl::PointNormal>::Ptr pointsPtr(new pcl::PointCloud<pcl::PointNormal>);
	pcl::fromPCLPointCloud2(mesh.cloud, *pointsPtr);
	for (std::size_t i = 0; i < mesh.polygons.size(); i++) {
		for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
			pcl::PointNormal point = pointsPtr->points[mesh.polygons[i].vertices[j]];
			glMeshData.emplace_back(point.x);
			glMeshData.emplace_back(point.y);
			glMeshData.emplace_back(point.z);
			glMeshData.emplace_back(point.normal_x);
			glMeshData.emplace_back(point.normal_y);
			glMeshData.emplace_back(point.normal_z);
		}
	}
}
void DataProcessing::ply2ply(std::string src, std::string dst) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(src.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> filter = vtkSmartPointer<vtkTriangleFilter>::New();
	filter->SetInputData(reader->GetOutput());

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dst.c_str());
	writer->SetInputConnection(filter->GetOutputPort());
	writer->SetFileTypeToASCII();
	writer->SetColorModeToOff();
	writer->Update();
	writer->Write();
}