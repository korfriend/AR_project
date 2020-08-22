#pragma once

#ifndef __dojostatic
#define __dojostatic extern "C" __declspec(dllexport)
#endif

#include <string>
#include <vector>
#include <bitset>

namespace optitrk
{
	__dojostatic bool InitOptiTrackLib();
	// video_type
	//==     0 = Segment Mode   
	//==     1 = Grayscale Mode 
	//==     2 = Object Mode    
	//==     4 = Precision Mode
	//==     6 = MJPEG Mode
	// exposure [0~480] and threshold [0~255]
	// intensity [0~15] almost 15
	__dojostatic bool SetCameraSettings(int cam_idx, int video_type, int exposure, int threshold, int intensity = 15);
	__dojostatic bool SetCameraFrameRate(int cam_idx, int frameRate);

	__dojostatic bool LoadProfileAndCalibInfo(const std::string& file_profile, const std::string& file_calib);
	__dojostatic int GetMarkersLocation(std::vector<float>* mk_xyz_list, std::vector<float>* mk_residual_list = NULL, std::vector<std::bitset<128>>* mk_cid_list = NULL);
	__dojostatic int GetRigidBodies(std::vector<std::string>* rb_names = NULL);
	// mat_rb2ws ==> glm::fmat4x4
	__dojostatic bool SetRigidBodyPropertyById(const int rb_idx, const float smooth_term, const int test_smooth_term);
	__dojostatic bool SetRigidBodyPropertyByName(const std::string& name, const float smooth_term, const int test_smooth_term);
	__dojostatic bool GetRigidBodyLocationById(const int rb_idx, float* mat_rb2ws, std::vector<float>* rbmk_xyz_list = NULL, std::vector<float>* trmk_xyz_list = NULL, std::vector<bool>* tr_list = NULL, std::string* rb_name = NULL);
	__dojostatic bool GetRigidBodyLocationByName(const std::string& name, float* mat_rb2ws, std::vector<float>* rbmk_xyz_list = NULL, std::vector<float>* trmk_xyz_list = NULL, std::vector<bool>* tr_list = NULL, int* rb_id = NULL);
	// mat_cam2ws ==> glm::fmat4x4
	// normal camera space.. up : y down (-y), view : +z
	__dojostatic bool GetCameraLocation(const int cam_idx, float* mat_cam2ws);
	__dojostatic bool UpdateFrame(bool use_latest = false);
	__dojostatic bool DeinitOptiTrackLib();
}