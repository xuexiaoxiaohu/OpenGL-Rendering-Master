#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"

DataProcessing::DataProcessing() {
	meshConvertCloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
	meshConvertNormals.reset(new pcl::PointCloud<pcl::Normal>);
	normalsRefinedPtr.reset(new pcl::PointCloud<pcl::Normal>);
}

DataProcessing::~DataProcessing() {

}

void DataProcessing::clearMeshData() {
	surfaceModelData.vecFaceTriangles.clear();
	surfaceModelData.vecVertexNormals.clear();
	surfaceModelData.vecPoints.clear();
	surfaceModelData.vecTemPoints.clear();
}

std::string DataProcessing::getAppPath() {
	QString qAppDir = QCoreApplication::applicationDirPath();
	std::string::size_type iPos = (qAppDir.toStdString().find_last_of('\\') + 1) == 0 ?qAppDir.toStdString().find_last_of('/') + 1 : qAppDir.toStdString().find_last_of('\\') + 1;
	return qAppDir.toStdString().substr(0, iPos);
}

void DataProcessing::getCenterPoint(QVector3D& vec) {
	//if (0 == pointData.size()) return;
	vec.setX((maxCoord.x() + minCoord.x()) / 2);
	vec.setY((maxCoord.y() + minCoord.y()) / 2);
	vec.setZ((maxCoord.z() + minCoord.z()) / 2);
}

void DataProcessing::centralizeOriginalPoints(std::vector<QVector3D> data) {
	/*pointData = data;
	getXYZMaxMin();

	getCenterPoint(centerPoint);
	for (int i = 0; i < pointData.size(); i++) {
		pointData[i].setX(pointData[i].x() - centerPoint.x());
		pointData[i].setY(pointData[i].y() - centerPoint.y());
		pointData[i].setZ(pointData[i].z() - centerPoint.z());
	}
	getXYZMaxMin();

	float max = 0;
	if (max <= maxCoord.x()) max = maxCoord.x();
	if (max <= maxCoord.y()) max = maxCoord.y();
	if (max <= maxCoord.z()) max = maxCoord.z();

	float factor = 1.0 / max;
	for (int i = 0; i < pointData.size(); i++)	pointData[i] = { pointData[i].x() * factor ,pointData[i].y() * factor ,pointData[i].z() * factor };*/
}
void DataProcessing::getMaxMinCoord(std::vector<QVector3D> data) {
	QVector3D vecMax, vecMin;
	vecMax = { data[0].x() ,data[0].y() ,data[0].z() };
	vecMin = vecMax;

	for (int i = 0; i < data.size(); i++) {
		if (vecMax.x() < data[i].x())	vecMax.setX(data[i].x());
		if (vecMax.y() < data[i].y())	vecMax.setY(data[i].y());
		if (vecMax.z() < data[i].z())	vecMax.setZ(data[i].z());

		if (vecMin.x() > data[i].x())	vecMin.setX(data[i].x());
		if (vecMin.y() > data[i].y())	vecMin.setY(data[i].y());
		if (vecMin.z() > data[i].z())	vecMin.setZ(data[i].z());
	}
	maxCoord = vecMax;
	minCoord = vecMin;
}

void DataProcessing::getXYZMaxMin() {
	//if (0 == pointData.size())	return;
	//QVector3D vecMax, vecMin;
	//vecMax = { pointData[0].x() ,pointData[0].y() ,pointData[0].z() };
	//vecMin = vecMax;

	//for (int i = 0; i < pointData.size(); i++) {
	//	if (vecMax.x() < pointData[i].x())	vecMax.setX(pointData[i].x());
	//	if (vecMax.y() < pointData[i].y())	vecMax.setY(pointData[i].y());
	//	if (vecMax.z() < pointData[i].z())	vecMax.setZ(pointData[i].z());

	//	if (vecMin.x() > pointData[i].x())	vecMin.setX(pointData[i].x());
	//	if (vecMin.y() > pointData[i].y())	vecMin.setY(pointData[i].y());
	//	if (vecMin.z() > pointData[i].z())	vecMin.setZ(pointData[i].z());
	//}
	//maxCoord = vecMax;
	//minCoord = vecMin;
}
void DataProcessing::ply2ply(std::string src, std::string dst) {
	vtkSmartPointer<vtkPLYReader> read = vtkSmartPointer<vtkPLYReader>::New();
	read->SetFileName(src.c_str());
	read->Update();

	vtkSmartPointer<vtkTriangleFilter> filter = vtkSmartPointer<vtkTriangleFilter>::New();
	filter->SetInputData(read->GetOutput());

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dst.c_str());
	writer->SetInputConnection(filter->GetOutputPort());
	writer->SetFileTypeToASCII();
	writer->SetColorModeToOff();
	writer->Update();
	writer->Write();
}
void DataProcessing::ply2pcd(std::string ply, std::string pcd) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr ply2pcdCloudPtr(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPLYFile<pcl::PointXYZ>(ply, *ply2pcdCloudPtr);
	pcl::io::savePCDFile(pcd, *ply2pcdCloudPtr);
}
void DataProcessing::meshConvert(std::string filename) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkSmartPointer<vtkTriangleFilter> filter = vtkSmartPointer<vtkTriangleFilter>::New();
	filter->SetInputData(reader->GetOutput());
	filter->Update();

	pcl::PolygonMesh mesh;
	pcl::io::vtk2mesh(filter->GetOutput(), mesh);

	pcl::fromPCLPointCloud2(mesh.cloud, *meshConvertCloud);

	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;

	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(meshConvertCloud);
	search.nearestKSearch(*meshConvertCloud, pcl::Indices(), NUM_NEIGHBORS, k_indices, k_sqr_distances);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;

	for (size_t i = 0; i < meshConvertCloud->size(); ++i) {
		pcl::Normal normal;
		ne.computePointNormal(*meshConvertCloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*meshConvertCloud)[i], (*meshConvertCloud).sensor_origin_[0], (*meshConvertCloud).sensor_origin_[1], (*meshConvertCloud).sensor_origin_[2],
			normal.normal_x, normal.normal_y, normal.normal_z);
		meshConvertNormals->emplace_back(normal);
	}
	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(meshConvertNormals);
	nr.setMaxIterations(ITERATIONS);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefinedPtr);

	writePlyData(mesh);
}
//Data Type Conversion(transfer mesh object to a file)
void DataProcessing::writePlyData(pcl::PolygonMesh mesh) {
	std::ofstream fs;
	fs.open("./finalMesh.ply");
	if (!fs)	PCL_ERROR("[pcl::io::savePLYFile] Error during opening (%s)!\n", "finalMesh.ply");

	int nr_points = mesh.cloud.width * mesh.cloud.height;
	if (nr_points == 0) return;
	// size of points
	int point_size = mesh.cloud.data.size() / nr_points;
	// number of faces
	int nr_faces = mesh.polygons.size();

	// Write header
	fs << "ply";
	fs << "\nformat ascii 1.0";
	fs << "\ncomment PCL generated";
	// Vertices
	fs << "\nelement vertex " << mesh.cloud.width * mesh.cloud.height;
	fs << "\nproperty float x"
		"\nproperty float y"
		"\nproperty float z";

	fs << "\nproperty float nx"
		"\nproperty float ny"
		"\nproperty float nz";
	// Faces
	fs << "\nelement face " << nr_faces;
	fs << "\nproperty list uchar int vertex_indices";
	fs << "\nend_header\n";

	for (std::size_t i = 0; i < nr_points; i++) {
		for (std::size_t d = 0; d < mesh.cloud.fields.size(); ++d) {
			// adding vertex
			if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
				mesh.cloud.fields[d].name == "x" ||
				mesh.cloud.fields[d].name == "y" ||
				mesh.cloud.fields[d].name == "z")) {
				float value;
				memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset], sizeof(float));
				fs << value << " ";
			}
		}
		fs << normalsRefinedPtr->points[i].normal_x << " ";
		fs << normalsRefinedPtr->points[i].normal_y << " ";
		fs << normalsRefinedPtr->points[i].normal_z << " ";
		fs << "\n";
	}
	// Write down faces
	for (std::size_t i = 0; i < nr_faces; i++) {
		fs << mesh.polygons[i].vertices.size() << " ";
		for (std::size_t j = 0; j < mesh.polygons[i].vertices.size() - 1; ++j)
			fs << mesh.polygons[i].vertices[j] << " ";
		fs << mesh.polygons[i].vertices.back() << '\n';
	}
	fs.close();
}
void DataProcessing::loadPointData(const char* path) {
	std::fstream readTextData(path);
	if (!readTextData) return;
	float x, y, z;
	while (readTextData >> x >> y >> z) {
		QVector3D data = { x, y, z };
		pointData.emplace_back(data);
	}
	readTextData.close();
}
//Mesh
void DataProcessing::loadMeshData(char* filename) {
	FILE* file = fopen(filename, "r");
	if (file) {
		fseek(file, 0, SEEK_END);
		clearMeshData();
		surfaceVertexXYZ = (float*)malloc(ftell(file));
		surfaceVertexNorm = (float*)malloc(ftell(file));
		fseek(file, 0, SEEK_SET);
		char buffer[MESH_BUFFER_MAX_SIZE];
		fgets(buffer, MESH_BUFFER_SIZE, file);
		// Find total vertex
		while (strncmp("element vertex", buffer, strlen("element vertex")) != 0) {
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}
		strcpy(buffer, buffer + strlen("element vertex"));
		sscanf(buffer, "%d", &this->surfaceTotalConnectedPoints);

		// Find total face
		fseek(file, 0, SEEK_SET);
		while (strncmp("element face", buffer, strlen("element face")) != 0) {
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}
		strcpy(buffer, buffer + strlen("element face"));
		sscanf(buffer, "%d", &this->surfaceTotalFaces);

		// go to end_header
		while (strncmp("end_header", buffer, strlen("end_header")) != 0) {
			fgets(buffer, MESH_BUFFER_SIZE, file);
		}

		// read vertices
		for (int iterator = 0, index = 0; iterator < this->surfaceTotalConnectedPoints; iterator++) {
			if (iterator == 0)	pointData.clear();
			fgets(buffer, MESH_BUFFER_SIZE, file);
			sscanf(buffer, "%f %f %f %f %f %f",
				&surfaceVertexXYZ[index], &surfaceVertexXYZ[index + 1], &surfaceVertexXYZ[index + 2],
				&surfaceVertexNorm[index], &surfaceVertexNorm[index + 1], &surfaceVertexNorm[index + 2]);

			surfaceModelData.vecTemPoints.emplace_back(surfaceVertexXYZ[index]);
			surfaceModelData.vecTemPoints.emplace_back(surfaceVertexXYZ[index + 1]);
			surfaceModelData.vecTemPoints.emplace_back(surfaceVertexXYZ[index + 2]);
			QVector3D data = { surfaceVertexXYZ[index] ,surfaceVertexXYZ[index + 1] ,surfaceVertexXYZ[index + 2] };
			pointData.emplace_back(data);
			index += 3;
		}
		centralizeOriginalPoints(pointData);
		for (int i = 0; i < pointData.size(); i++) {
			surfaceModelData.vecPoints.emplace_back(pointData[i].x());
			surfaceModelData.vecPoints.emplace_back(pointData[i].y());
			surfaceModelData.vecPoints.emplace_back(pointData[i].z());
		}

		// read faces
		for (int iterator = 0; iterator < this->surfaceTotalFaces; iterator++) {
			fgets(buffer, MESH_BUFFER_SIZE, file);
			if (buffer[0] == '3') {
				int vertex1 = 0, vertex2 = 0, vertex3 = 0;
				buffer[0] = ' ';

				//read the index of the three vertices that make up the face
				sscanf(buffer, "%d %d %d", &vertex1, &vertex2, &vertex3);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex1]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex1 + 1]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex1 + 2]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex2]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex2 + 1]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex2 + 2]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex3]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex3 + 1]);
				surfaceModelData.vecFaceTriangles.emplace_back(surfaceModelData.vecPoints[3 * vertex3 + 2]);

				// Normal
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1 + 1]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex1 + 2]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2 + 1]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex2 + 2]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3 + 1]);
				surfaceModelData.vecVertexNormals.emplace_back(surfaceVertexNorm[3 * vertex3 + 2]);
			}
		}

		free(surfaceVertexXYZ);		surfaceVertexXYZ = NULL;
		free(surfaceVertexNorm);	surfaceVertexNorm = NULL;
		fclose(file);
	}
}
// Get Data From Mesh
void DataProcessing::getMeshData(pcl::PolygonMesh mesh) {
	if (mesh.cloud.data.empty())	PCL_ERROR("[pcl::io::savePLYFile] Input point cloud has no data!\n");
	// number of points
	int nr_points = mesh.cloud.width * mesh.cloud.height;
	// size of points
	int point_size = mesh.cloud.data.size() / nr_points;
	// number of faces
	int nr_faces = mesh.polygons.size();

	for (std::size_t i = 0; i < nr_points; i++) {
		if (i == 0)	meshVertex1D.clear();
		for (std::size_t d = 0; d < mesh.cloud.fields.size(); ++d) {
			// adding vertex
			if ((mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (mesh.cloud.fields[d].name == "x" ||mesh.cloud.fields[d].name == "y" ||mesh.cloud.fields[d].name == "z")) {
				float value;
				memcpy(&value, &mesh.cloud.data[i * point_size + mesh.cloud.fields[d].offset], sizeof(float));
				meshVertex1D.emplace_back(value);
			}
		}
	}

	for (int i = 0, index = 0; i < meshVertex1D.size() / 3; i++) {
		if (i == 0)	meshVertex3D.resize(meshVertex1D.size() / 3);
		meshVertex3D[i].setX(meshVertex1D[index]);
		meshVertex3D[i].setY(meshVertex1D[index + 1]);
		meshVertex3D[i].setZ(meshVertex1D[index + 2]);
		index += 3;
	}
	centralizeOriginalPoints(meshVertex3D);

	for (std::size_t i = 0; i < nr_faces; i++) {
		if (i == 0) meshData.clear();
		for (std::size_t j = 0; j < mesh.polygons[i].vertices.size(); j++) {
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].x());
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].y());
			meshData.emplace_back(pointData[mesh.polygons[i].vertices[j]].z());
		}
	}
}

// get normal vector of point cloud
void DataProcessing::getNormalVector(std::string pcdPath) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcdPath, *cloud) == -1) PCL_ERROR("Could not read file\n");

	std::vector<pcl::Indices> k_indices;
	std::vector<std::vector<float>> k_sqr_distances;
	pcl::search::KdTree<pcl::PointXYZ> search;
	search.setInputCloud(cloud);
	search.nearestKSearch(*cloud, pcl::Indices(), NUM_NEIGHBORS, k_indices, k_sqr_distances);
	pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	for (size_t i = 0; i < cloud->size(); ++i) {
		pcl::Normal normal;
		ne.computePointNormal(*cloud, k_indices[i], normal.normal_x, normal.normal_y, normal.normal_z, normal.curvature);
		pcl::flipNormalTowardsViewpoint((*cloud)[i], (*cloud).sensor_origin_[0], (*cloud).sensor_origin_[1],
			(*cloud).sensor_origin_[2], normal.normal_x, normal.normal_y, normal.normal_z);
		normals->emplace_back(normal);
	}
	pcl::NormalRefinement<pcl::Normal> nr(k_indices, k_sqr_distances);
	nr.setInputCloud(normals);
	nr.setMaxIterations(ITERATIONS);
	nr.setConvergenceThreshold(0.1);
	nr.filter(*normalsRefinedPtr);
}