#pragma once
#include <vector>
#include <set>
#include <string>

struct __MarkerDetInfo
{
	int id;
	//double confidence;
	std::vector< float > corners2d;

	__MarkerDetInfo() {
		id = -1;// , confidence = -1;
	};
};

class __declspec(dllexport) ArMarkerTracker // must use this as a singleton class
{
public:
	// the markers are pre-defined in this constructor as Aruco markers
	// if you want to check the specific markers, use 'aruco_marker_file_out' to output image file
	ArMarkerTracker();
	~ArMarkerTracker();

	// markerid should be [0, 49] pre-defined by Aruco
	void aruco_marker_file_out(const int markerId, const std::string& filename);


	// :: register a marker size for pose estimation
	// markerId : marker id pre-defined in constructor
	// return value 0 : works okay, !0 L error
	int register_marker(const int markerId, const double marker_width);

	// :: track the specified (by id) markers
	// list_det_markers : output of the tracking process
	// return value : > 0 ==> # of detected markers (same as list_det_markers.size()), < 0 ==> error exists
	int track_markers(std::vector<__MarkerDetInfo>& list_det_markers,
		const unsigned char* gray_img, const int w, const int h, const std::set<int>& marker_IDs);
};