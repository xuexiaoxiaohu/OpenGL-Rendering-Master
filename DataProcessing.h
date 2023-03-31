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

struct SurfaceModelData {
	std::vector<float> vecTemPoints;
	std::vector<float> vecPoints;
	std::vector<float> vecFaceTriangles;
	std::vector<float> vecVertexNormals;
};

class DataProcessing {
public:
	DataProcessing();
	~DataProcessing();
	void getMeshData(pcl::PolygonMesh mesh);
	void loadPointData(const char* path);
	void loadMeshData(char* filename);
	void meshConvert(std::string filename);
	void writePlyData(pcl::PolygonMesh mesh);
	void centralizeOriginalPoints(std::vector<QVector3D> data);
	std::string getAppPath();
	void getNormalVector(std::string pcdPath);
	void ply2ply(std::string src, std::string dst);
	void ply2pcd(std::string ply, std::string pcd);

	// point data
	std::vector<QVector3D>	pointData;
	std::vector<float>		meshData;
	SurfaceModelData		surfaceModelData;

	void getMaxMinCoord(std::vector<QVector3D> data);

	QVector3D centerPoint;
	QVector3D maxCoord;
	QVector3D minCoord;
private:
	void clearMeshData();
	void getXYZMaxMin();

	void getCenterPoint(QVector3D& vec);

	std::vector<QVector3D> meshVertex3D;
	std::vector<float> lastMeshVertex1D;
	float* surfaceVertexXYZ; 
	float* surfaceVertexNorm;
	int surfaceTotalConnectedPoints;
	int surfaceTotalFaces;
	std::vector<float> meshVertex1D;
	std::vector<QVector3D> normal;

	pcl::PointCloud<pcl::PointXYZ>::Ptr meshConvertCloud;
	pcl::PointCloud<pcl::Normal>::Ptr meshConvertNormals;
	pcl::PointCloud<pcl::Normal>::Ptr normalsRefinedPtr;
};