#include "DataProcessing.h"
#include <QApplication>
#include "Macro.h"
#include <qDebug>
#include <QFile>
#include <QDataStream>

void DataProcessing::loadPointData(QString path) {
	QFile file(path);
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))	return;

	QTextStream in(&file);
	while (!in.atEnd()) {
		QString line = in.readLine();
		QStringList parts = line.split(" ");
		QVector3D vector(parts[0].toFloat(), parts[1].toFloat(), parts[2].toFloat());
		pointData.append(vector);
	}
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
	
void DataProcessing::isoExpRemeshing(const char* srcPath, const char* dstPath) {
	vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
	reader->SetFileName(srcPath);
	reader->Update();
	auto polyData = reader->GetOutput();

	vtkSmartPointer<vtkImageData> whiteImage = vtkSmartPointer<vtkImageData>::New();
	double bounds[6];
	polyData->GetBounds(bounds);
	double spacing[3];
	spacing[0] = 0.5;
	spacing[1] = 0.5;
	spacing[2] = 0.5;

	int dim[3];
	for (int i = 0; i < 3; i++) {
		dim[i] = static_cast<int>(ceil((bounds[i * 2 + 1] - bounds[i * 2]) / spacing[i]));
	}

	whiteImage->SetDimensions(dim);
	whiteImage->SetExtent(0, dim[0] - 1, 0, dim[1] - 1, 0, dim[2] - 1);

	double origin[3];
	origin[0] = bounds[0] + spacing[0] / 2;
	origin[1] = bounds[2] + spacing[1] / 2;
	origin[2] = bounds[4] + spacing[2] / 2;
	whiteImage->SetOrigin(origin);
	whiteImage->SetSpacing(spacing);
	whiteImage->AllocateScalars(VTK_UNSIGNED_CHAR, 1);

	unsigned char inval = 255;
	unsigned char outval = 0;
	vtkIdType count = whiteImage->GetNumberOfPoints();
	for (vtkIdType i = 0; i < count; ++i) {
		whiteImage->GetPointData()->GetScalars()->SetTuple1(i, inval);
	}
	
	vtkSmartPointer<vtkPolyDataToImageStencil> pol2stenc = vtkSmartPointer<vtkPolyDataToImageStencil>::New();
	pol2stenc->SetInputData(polyData);
	pol2stenc->SetOutputOrigin(origin);
	pol2stenc->SetOutputSpacing(spacing);
	pol2stenc->SetOutputWholeExtent(whiteImage->GetExtent());
	pol2stenc->Update();

	vtkSmartPointer<vtkImageStencil> imgstenc = vtkSmartPointer<vtkImageStencil>::New();
	imgstenc->SetInputData(whiteImage);
	imgstenc->SetStencilConnection(pol2stenc->GetOutputPort());
	imgstenc->ReverseStencilOff();
	imgstenc->SetBackgroundValue(outval);
	imgstenc->Update();

	vtkSmartPointer<vtkImageGaussianSmooth> gaussianSmooth = vtkSmartPointer<vtkImageGaussianSmooth>::New();
	gaussianSmooth->SetInputConnection(imgstenc->GetOutputPort());
	gaussianSmooth->SetDimensionality(3);
	gaussianSmooth->SetRadiusFactor(5);
	gaussianSmooth->SetStandardDeviation(1);
	gaussianSmooth->Update();

	vtkSmartPointer<vtkMarchingCubes>marchingcube = vtkSmartPointer<vtkMarchingCubes>::New();
	marchingcube->SetInputData(gaussianSmooth->GetOutput());
	marchingcube->SetValue(0, 70);
	marchingcube->ComputeNormalsOn();
	marchingcube->Update();

	vtkSmartPointer<vtkPLYWriter> writer = vtkSmartPointer<vtkPLYWriter>::New();
	writer->SetFileName(dstPath);
	writer->SetInputData(marchingcube->GetOutput());
	writer->SetFileTypeToASCII();
	writer->Update();
}