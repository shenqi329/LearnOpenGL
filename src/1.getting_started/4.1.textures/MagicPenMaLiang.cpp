#include "MagicPenMaLiang.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "polypartition.h"

// MagicPenModel3D begin

void MagicPen3DModel::Init(std::list<TPPLPoly> triangles, std::vector<cv::Point> contour, int cols, int rows,  int texture_side_width, int texture_side_height) {
	

	FreeVertice();

	InitVerticesFront(triangles, cols, rows);
	
	InitVerticesEdge(contour, cols, rows, texture_side_width, texture_side_height);
}

void MagicPen3DModel::InitVerticesFront(std::list<TPPLPoly> triangles, int cols, int rows) {

	_vertices_front_size = sizeof(Vertice) * 3 * triangles.size();
	_vertices_front = (Vertice*)malloc(_vertices_front_size);

	_indices_front_size = sizeof(int) * 3 * triangles.size();
	_indices_front = (int*)malloc(_indices_front_size);

	int index = 0;
	for(std::list<TPPLPoly>::iterator iter = triangles.begin() ; iter != triangles.end() ;iter++, index++) {
		
		if(3 != iter->GetNumPoints()) {
			continue;
		}

		for (size_t i = 0; i < 3; i++) {

			int offset = (index*3 + i);

			// position
			_vertices_front[offset].positions[0] =  (iter->GetPoints()[i].x - cols/2) * 2 / cols;
			_vertices_front[offset].positions[1] = -(iter->GetPoints()[i].y - rows/2) * 2 / rows;
			_vertices_front[offset].positions[2] = 0.0f;

			// color
			_vertices_front[offset].colors[0] = 0.5f;
			_vertices_front[offset].colors[1] = 0.5f;
			_vertices_front[offset].colors[2] = 0.5f;

			// texture coords
			_vertices_front[offset].textures[0] = iter->GetPoints()[i].x / cols;
			_vertices_front[offset].textures[1] = iter->GetPoints()[i].y / rows;

			_indices_front[index*3 + i] = index*3 + i;
		}
	}

}

void MagicPen3DModel::InitVerticesEdge(std::vector<cv::Point> contour, int cols, int rows, int texture_side_width, int texture_side_height) {

	_vertices_side_size = sizeof(Vertice) * 2 * contour.size();
	_vertices_side = (Vertice*)malloc(_vertices_side_size);

	_indices_side_size = sizeof(int) * 3 * 2 * contour.size();
	_indices_side = (int*)malloc(_indices_side_size);

	float distance_total = 0;
	for (int index = 0; index < contour.size(); index++) {

		int point_next_index = (index + 1) % contour.size();

		float point_x =  (float(contour[index].x) - cols/2) * 2 / cols;
		float point_y = -(float(contour[index].y) - rows/2) * 2 / rows;

		float point_next_x =  (float(contour[point_next_index].x) - cols/2) * 2 / cols;
		float point_next_y = -(float(contour[point_next_index].y) - rows/2) * 2 / rows;

		float x_distance = point_x - point_next_x;
		float y_distance = point_y - point_next_y;
		float distance = sqrtf(x_distance*x_distance + y_distance * y_distance);
		distance = distance * texture_side_width / 0.1f / texture_side_height;

		distance_total += distance;
		for (int i = 0; i < 2; i++) {
			int offset = (index*2 + i);

			_vertices_side[offset].positions[0] =  point_x;
			_vertices_side[offset].positions[1] =  point_y;

			if(0 == i) {
				_vertices_side[offset].positions[2] = 0.0f;
				_vertices_side[offset].textures[0] = 0;  //  texture coords x
			} else {
				_vertices_side[offset].positions[2] = -0.1f;
				_vertices_side[offset].textures[0] = 1.0f; // texture coords x
			}
			_vertices_side[offset].textures[1] = distance_total;  //texture coords y

			// color
			_vertices_side[offset].colors[0] = 0.5f;
			_vertices_side[offset].colors[1] = 0.5f;
			_vertices_side[offset].colors[2] = 0.5f;
		}
		
		if (distance_total >= 0.5f) {
			distance_total = 0.0f;
		}

		int offset = index * 3 * 2;
		// first triangle 
		_indices_side[offset + 0] = index * 2;
		_indices_side[offset + 1] = index * 2 + 1;
		_indices_side[offset + 2] = point_next_index * 2;

		// second triangle
		_indices_side[offset + 3] = index * 2 + 1;
		_indices_side[offset + 4] = point_next_index * 2;
		_indices_side[offset + 5] = point_next_index * 2 + 1;
	}
}

MagicPen3DModel::~MagicPen3DModel() {

	FreeVertice();

}

void MagicPen3DModel::FreeVertice() {

	// vertices front
	if(_vertices_front) {
		free(_vertices_front);
		_vertices_front = nullptr;
		_vertices_front_size = 0;
	}

	if(_indices_front) {
		free(_indices_front);
		_indices_front = nullptr;
		_indices_front_size = 0;
	}

	// vertices edge
	if (_vertices_side) {
		free(_vertices_side);
		_vertices_side = nullptr;
		_vertices_side_size = 0;
	}

	if (_indices_side) {
		free(_indices_side);
		_indices_side = nullptr;
		_indices_side_size = 0;
	}
}

// MagicPenModel3D end


// MagicPenMaLiang begin
static void ApproxPoly(std::vector<std::vector<cv::Point> > &contours) {
	
	for (size_t i = 0; i < contours.size(); i++) {
		std::vector<cv::Point> outContour;
		cv::approxPolyDP(contours[i], outContour, 1, false);
		contours[i] = outContour;
	}
}

static bool IsLineBreak(cv::Mat &detected_edges, int row, int col) {

    int count = 0;

    int first_i;
    int first_j;
    bool near_first_second = false;

    for (int i = -1; i <= 1; i++) {
        for (int j = -1; j <= 1; j++) {
            
            if (255 == detected_edges.at<uchar>(row + i, col +j)) {
                if (i == 0 && j == 0) {
                    continue;
                }
                if (0 == count) {
                    first_i = i;
                    first_j = j;
                } else if (1 == count) {
                    near_first_second = (abs(first_i - i) + abs(first_j - j)) <= 1;
                }
                count++;    
            }
        }
    }

    if(count <= 1) {
        return true;
    }

    if (count == 2) {
        return near_first_second;
    }

    return false;
}


bool MagicPenMaLiang::Magic(cv::Mat image, int texture_side_width, int texture_side_height) {

	cv::Mat image_gray;
	cv::Mat	detected_edges;

	cvtColor(image, image_gray, cv::COLOR_BGR2GRAY);

    //![reduce_noise]
    /// Reduce noise with a kernel 3x3
    cv::blur(image_gray, detected_edges, cv::Size(3,3));
    //![reduce_noise]

    //![canny]
    /// Canny detector
    cv::Canny(detected_edges, detected_edges, threshold, threshold*ratio, kernel_size);
    //![canny]

    ConnectAdjacentEdge(detected_edges);

    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(detected_edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	if (contours.size() <= 0) {
		return  false;
	}

	ApproxPoly(contours);

	float maxArea = 0;
	int maxAreaIndex = 0;
    for (size_t i = 0; i < contours.size(); i++) {
		cv::RotatedRect minAreaRect = cv::minAreaRect(contours[i]);
		
		float area = minAreaRect.size.width * minAreaRect.size.height;
		if (area > maxArea) {
			maxAreaIndex = i;
		}
    }

	PolyTriangulate(detected_edges,  contours[maxAreaIndex]);
	_3dModel.Init(_result, contours[maxAreaIndex], image.cols, image.rows, texture_side_width, texture_side_height);

#ifdef MagicPenMaLiang_DEBUG
    // Create the marker image for the watershed algorithm
    cv::Mat markers = cv::Mat::zeros(detected_edges.size(), CV_8U);

	drawContours(markers, contours, static_cast<int>(maxAreaIndex), cv::Scalar(255), -1);

	// Draw the background marker
    cv::imshow("Markers", markers);

    //![display]
    cv::imshow("Edge Map", detected_edges);
    //![display]
#endif

	return true;
}

MagicPen3DModel *MagicPenMaLiang::Get3DModel()
{
	return &_3dModel;
}

void MagicPenMaLiang::PolyTriangulate(cv::Mat &detected_edges, std::vector<cv::Point> &contour) {

    TPPLPartition pp;

    TPPLPoly poly;
    poly.Init(contour.size());
    poly.SetHole(false);

    for (size_t i = 0; i < contour.size(); i++) {
        poly[i].x = contour[i].x;
        poly[i].y = contour[i].y;
    }
    poly.SetOrientation(TPPL_ORIENTATION_CCW);
    pp.Triangulate_OPT(&poly, &_result);


#ifdef MagicPenMaLiang_DEBUG
    // Create the marker image for the watershed algorithm
    cv::Mat triangulate = cv::Mat::zeros(detected_edges.size(), CV_8U);

    std::list<TPPLPoly>::iterator iter;

    for(iter = _result.begin(); iter != _result.end() ;iter++) {
        if(3 == iter->GetNumPoints()) {
            cv::Point point_a(iter->GetPoints()[0].x, iter->GetPoints()[0].y);
            cv::Point point_b(iter->GetPoints()[1].x, iter->GetPoints()[1].y);
            cv::Point point_c(iter->GetPoints()[2].x, iter->GetPoints()[2].y);
            cv::line(triangulate, point_a, point_b, cv::Scalar(255));
            cv::line(triangulate, point_b, point_c, cv::Scalar(255));
            cv::line(triangulate, point_c, point_a, cv::Scalar(255));
        }
    }

    cv::imshow("Triangulate", triangulate);
#endif
}

void MagicPenMaLiang::ConnectAdjacentEdge(cv::Mat &detected_edges) {
	for (int row = 1; row < detected_edges.rows - 1; row++) {

		for (int col = 1; col < detected_edges.cols - 1; col++) {
      
			if(0 == detected_edges.at<uchar>(row, col)) {
				continue;
			}

			if(!IsLineBreak(detected_edges, row, col)) {
				continue; 
			}

			for (int i = -1; i <= 1; i++) {
				for (int j = -1; j <= 1; j++) {
					detected_edges.at<uchar>(row + i, col +j) = 255;
				}
			}
		}
	}
}
// MagicPenMaLiang end