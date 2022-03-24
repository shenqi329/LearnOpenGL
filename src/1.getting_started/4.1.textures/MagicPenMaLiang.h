#ifndef MAGIC_PEN_MALIANG_H
#define MAGIC_PEN_MALIANG_H

#include <opencv2/imgproc.hpp>
#include "polypartition.h"

#define MagicPenMaLiang_DEBUG

// 顶点
struct Vertice {
	float positions[3]; // x,y,z 坐标
	float colors[3];	// 颜色
	float textures[2];  // 纹理坐标
};

// 3D模型
class MagicPen3DModel {
public:
	void Init(std::list<TPPLPoly> triangles, std::vector<cv::Point> contour, int cols, int rows, int texture_side_width, int texture_side_height);

	~MagicPen3DModel();
private:
	void InitVerticesFront(std::list<TPPLPoly> triangles, int cols, int rows);

	void InitVerticesEdge(std::vector<cv::Point> contour, int cols, int rows, int texture_side_width, int texture_height);

	void FreeVertice();

public:
	// 正面信息
	Vertice *_vertices_front = nullptr;
	int _vertices_front_size = 0;

	int _indices_front_size = 0;
	int* _indices_front = nullptr;

	// 侧面信息
	Vertice *_vertices_side = nullptr;
	int _vertices_side_size = 0;

	int _indices_side_size = 0;
	int* _indices_side = nullptr;
};

class MagicPenMaLiang
{
public:
	bool Magic(cv::Mat image, int texture_side_width, int texture_side_height);

	MagicPen3DModel *Get3DModel();
private:
	
	// 连接临近的edge
	void ConnectAdjacentEdge(cv::Mat &detected_edges);

	// 计算填充满多边形的三角形
	void PolyTriangulate(cv::Mat &detected_edges, std::vector<cv::Point> &contour);

private:
	std::list<TPPLPoly> _result;

	MagicPen3DModel _3dModel;
	static const int threshold = 100;
	static const int ratio = 3;
	static const int kernel_size = 3;
};

#endif

