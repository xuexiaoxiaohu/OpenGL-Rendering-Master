#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"

std::string DataProcessing::getAppPath() {
	QString qAppDir = QCoreApplication::applicationDirPath();
	std::string::size_type iPos = (qAppDir.toStdString().find_last_of('\\') + 1) == 0 ?
		qAppDir.toStdString().find_last_of('/') + 1 : qAppDir.toStdString().find_last_of('\\') + 1;
	return qAppDir.toStdString().substr(0, iPos);
}
void DataProcessing::loadPointData(const char* path) {
	std::fstream fs(path);
	if (fs.is_open() == NULL) return;
	float x, y, z;
	while (fs >> x >> y >> z) pointData.emplace_back(QVector3D {x, y, z});
	fs.close();
}

void DataProcessing::getMaxMinPoint(std::vector<QVector3D> data) {
	maxPoint = minPoint = {data[0].x() ,data[0].y() ,data[0].z()};
	for (int i = 0; i < data.size(); i++) {
		if (maxPoint.x() < data[i].x())	maxPoint.setX(data[i].x());
		if (maxPoint.y() < data[i].y())	maxPoint.setY(data[i].y());
		if (maxPoint.z() < data[i].z())	maxPoint.setZ(data[i].z());

		if (minPoint.x() > data[i].x())	minPoint.setX(data[i].x());
		if (minPoint.y() > data[i].y())	minPoint.setY(data[i].y());
		if (minPoint.z() > data[i].z())	minPoint.setZ(data[i].z());
	}
}

void DataProcessing::addNV(pcl::PolygonMesh &mesh) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;
	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud);
	search.nearestKSearch(*cloud, pcl::Indices(), NUM_NEIGHS, k_indices, k_sqr_distances);

	pcl::PointCloud<pcl::Normal>::Ptr normalsPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::PointCloud<pcl::Normal>::Ptr normalsRefinedPtr(new pcl::PointCloud<pcl::Normal>);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	for (size_t i = 0; i < cloud->size(); ++i) {
		pcl::Normal normal;
		ne.computePointNormal(*cloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*cloud)[i], (*cloud).sensor_origin_[0], (*cloud).sensor_origin_[1],
			(*cloud).sensor_origin_[2], normal.normal_x, normal.normal_y, normal.normal_z);
		normalsPtr->emplace_back(normal);
	}

	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normalsPtr);
	nr.setMaxIterations(ITERATS);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefinedPtr);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normalsRefinedPtr, *cloud_with_normals);
	pcl::PCLPointCloud2 outputCloud;
	pcl::toPCLPointCloud2(*cloud_with_normals, outputCloud);
	mesh.cloud = outputCloud;
}

int DataProcessing::getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVtx) {
	int indexVertex = -1;
	float minDist = std::numeric_limits<float>::max();
	for (int i = 0; i < glVtx.size(); i++) {
		float dist = (glVtx[i] - worldPos).length();
		if (dist < minDist) {
			indexVertex = i;
			minDist = dist;
		}
	}
	return indexVertex;
}
std::vector<int> DataProcessing::radiusSearch(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint, float radius) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

	pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	kdtree->setInputCloud(cloud);

	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;

	kdtree->radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance);

	return pointIdxRadiusSearch;
}
void DataProcessing::eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete) {
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
}
void DataProcessing::fillMesh(pcl::PolygonMesh& mesh) {
	vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
	vtkSmartPointer<vtkFillHolesFilter> fillHoles = vtkSmartPointer<vtkFillHolesFilter>::New();
	pcl::io::mesh2vtk(mesh, polydata);
	fillHoles->SetInputData(polydata);
	fillHoles->SetHoleSize(100.0);
	fillHoles->Update();
	pcl::io::vtk2mesh(fillHoles->GetOutput(), mesh);
}

void DataProcessing::getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh,
	std::vector<QVector3D> vertices, float radius){
	int index = getNearestVertexIndex(worldPos, vertices);
	if (index != -1) {
		pcl::PointXYZ nrstVertex;
		nrstVertex.x = vertices[index].x();
		nrstVertex.y = vertices[index].y();
		nrstVertex.z = vertices[index].z();

		pcl::Indices toRemove = radiusSearch(mesh, nrstVertex, radius);
		eraseMesh(mesh, toRemove);
		fillMesh(mesh);
		getRenderData(mesh);
	}
}
void DataProcessing::getRenderData(pcl::PolygonMesh &mesh) {
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
void DataProcessing::poly2tri(std::string src, std::string dst) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(src.c_str());
	reader->Update();

	vtkSmartPointer<vtkTriangleFilter> filter = vtkSmartPointer<vtkTriangleFilter>::New();
	filter->SetInputData(reader->GetOutput());

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dst.c_str());
	writer->SetInputConnection(filter->GetOutputPort());
	writer->SetFileTypeToASCII();
	writer->Update();
}