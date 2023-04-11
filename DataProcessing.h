#pragma once
// QT
#include <QVector3D>
// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/io/ply/ply.h>
#include <pcl/PolygonMesh.h>
#include <pcl/filters/normal_refinement.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/vtk_lib_io.h>
//VTK
#include <vtkSTLReader.h>
#include <vtkSTLWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkTriangleFilter.h>

class DataProcessing {
public:
	DataProcessing();
	~DataProcessing();

	std::string getAppPath();

	void loadPointData(const char* path);
	void addNormalForMesh(pcl::PolygonMesh &inMesh, pcl::PolygonMesh &outMesh);
	void getMaxMinCoord(std::vector<QVector3D> data);
	void ply2ply(std::string src, std::string dst);

	int findNearestVertex(QVector3D worldPos, std::vector<QVector3D> glMeshVertices);
	std::vector<int> findKNeighbors(pcl::PolygonMesh mesh, pcl::PointXYZ query_point);
	pcl::PolygonMesh eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete);
	void getDataAfterErase(QVector3D worldPos, pcl::PolygonMesh &mesh, std::vector<QVector3D> allVertices);
	void getGLMeshData(pcl::PolygonMesh &mesh);
	std::vector<QVector3D>	pointData;
	QVector3D maxCoord;
	QVector3D minCoord;

	//pcl::PolygonMesh mesh;
	//std::vector<QVector3D> allVertices;

	std::vector<float> glMeshData;
};