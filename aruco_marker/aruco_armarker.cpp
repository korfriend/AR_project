#include "aruco_armarker.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>

#include <map>
#define _USE_MATH_DEFINES
#include <math.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace cv;
using namespace std;

struct DetMarkerPrimitives
{
	// previous detection results //
	vector< int > id_markers;
	vector< vector< Point2f > > corners_set;

	void clear()
	{
		id_markers.clear();
		corners_set.clear();
	}
};

struct ArArucoMarkerInst
{
	Ptr<aruco::DetectorParameters> detectorParams;
	map<int, double> markers_length;

	DetMarkerPrimitives prev_markers;

	ArArucoMarkerInst() {
		detectorParams = aruco::DetectorParameters::create();
		detectorParams->cornerRefinementMethod =
			aruco::CORNER_REFINE_CONTOUR;// |
		//aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers ==> 너무 fluctuation 이 심하게 나타남
	}
};

bool __initialized = false;
Ptr<aruco::Dictionary> g_dictionary;
std::map<ArMarkerTracker*, ArArucoMarkerInst> g_map_aruco_marker_trackers;

ArMarkerTracker::ArMarkerTracker()
{
	if (!__initialized)
	{
		g_dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
		__initialized = true;
	}
	g_map_aruco_marker_trackers.insert(pair< ArMarkerTracker*, ArArucoMarkerInst>(this, ArArucoMarkerInst()));
};

ArMarkerTracker::~ArMarkerTracker()
{
	g_map_aruco_marker_trackers.erase(this);
};

void ArMarkerTracker::aruco_marker_file_out(const int markerId, const std::string& filename)
{
	Mat markerImg;
	aruco::drawMarker(g_dictionary, markerId, 512, markerImg, 1);
	imwrite(filename, markerImg);
}

int ArMarkerTracker::register_marker(const int markerId, const double marker_width)
{
	ArArucoMarkerInst& marker_inst = g_map_aruco_marker_trackers[this];
	marker_inst.markers_length[markerId] = marker_width * 0.001; // convert to meters
	return 0;
}

int ArMarkerTracker::track_markers(std::vector<__MarkerDetInfo>& list_det_markers,
	const unsigned char* gray_img, const int w, const int h, const std::set<int>& marker_IDs)
{
	ArArucoMarkerInst& marker_inst = g_map_aruco_marker_trackers[this];
	marker_inst.prev_markers.clear();
	if (marker_inst.markers_length.size() == 0) return 0;

	Mat image(h, w, CV_8UC1, (unsigned char*)gray_img);

	vector< int > ids;
	vector< vector< Point2f > > corners, rejected;

	// detect markers and estimate pose
	aruco::detectMarkers(image, g_dictionary, corners, ids, marker_inst.detectorParams, rejected);
	if (ids.size() == 0) return 0;

	for (size_t i = 0; i < ids.size(); i++)
	{
		int id = ids[i];
		if (marker_IDs.find(id) == marker_IDs.end()) continue;
		if (marker_inst.markers_length.find(id) == marker_inst.markers_length.end()) continue;

		__MarkerDetInfo marker_info;
		marker_info.id = id;

		vector< Point2f >& cornor_pts = corners[i];
		vector< Point2f > corners_single_marker;
		for (int j = 0; j < 4; j++)
		{
			corners_single_marker.push_back(cornor_pts[j]);
			marker_info.corners2d.push_back(cornor_pts[j].x);
			marker_info.corners2d.push_back(cornor_pts[j].y);
		}
		//marker_info.confidence = 0.9; // ??

		marker_inst.prev_markers.id_markers.push_back(id);
		marker_inst.prev_markers.corners_set.push_back(corners_single_marker);

		list_det_markers.push_back(marker_info);
	}

	return (int)list_det_markers.size();
}