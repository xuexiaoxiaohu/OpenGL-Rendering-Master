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
#include <vtkTriangleFilter.h>
#include <vtkFillHolesFilter.h>
#include <vtkPLYReader.h>
#include <vtkSmartPointer.h>
#include <vtkImageData.h>
#include <vtkPointData.h>
#include <vtkUnsignedCharArray.h>
#include <vtkSmartPointer.h>
#include <vtkXMLImageDataWriter.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyDataToImageStencil.h>
#include <vtkImageStencil.h>
#include <vtkMetaImageWriter.h>
#include <vtkMarchingCubes.h>
#include <vtkImageGaussianSmooth.h>
#include <vtkStripper.h>

class DataProcessing {
public:
	DataProcessing() {};
	~DataProcessing() {};

	void loadPointData(const char* path);
	void addNV(pcl::PolygonMesh &inMesh);
	void getMaxMinPoint(std::vector<QVector3D> data);
	void poly2tri(std::string src, std::string dst);

	int getNearestVertexIndex(QVector3D worldPos, std::vector<QVector3D> glVtx);
	std::vector<int> radiusSearch(pcl::PolygonMesh mesh, pcl::PointXYZ searchPoint, float radius = 4);
	void eraseMesh(pcl::PolygonMesh &mesh, std::vector<int> verticesToDelete);
	void fillMesh(pcl::PolygonMesh& mesh);
	void getErasedMesh(QVector3D worldPos, pcl::PolygonMesh &mesh, std::vector<QVector3D> vertices, float radius);
	void getRenderData(pcl::PolygonMesh &mesh);
	//Remeshing: Isotropic Explicit Remeshing
	void isoExpRemeshing(const char* srcPath,const char* dstPath);
	std::vector<QVector3D>	pointData;
	std::vector<float> glMeshData;
	QVector3D maxPoint,minPoint;
};