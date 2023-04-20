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
#include <vtkFillHolesFilter.h>

class DataProcessing {
public:
	DataProcessing() {};
	~DataProcessing() {};

	std::string getAppPath();
	void loadPointData(const char* path);
	void addNV(pcl::PolygonMesh &inMesh);
	void getMaxMinPoint(std::vector<QVector3D> data);
	void poly2tri(std::string src, std::string dst);

	int getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVtx);
	std::vector<int> findKNeighbors(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint);
	void eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete);
	void fillMesh(pcl::PolygonMesh& mesh);
	void getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, std::vector<QVector3D> vertices);
	void getRenderData(pcl::PolygonMesh &mesh);

	std::vector<QVector3D>	pointData;
	std::vector<float> glMeshData;
	QVector3D maxPoint;
	QVector3D minPoint;
};