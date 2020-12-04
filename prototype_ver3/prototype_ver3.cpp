#include <iostream>
#include <fstream>
#include <sstream>
//#include <vector>
//#include <string>
//#include <typeinfo>
//#include <unordered_map>
//#include <vector>
#include <windows.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "VisMtvApi.h"
#include "../optitrk/optitrack.h"

//#define EYE_VIS_RS
//#define EYE_VIS_RS_THREAD
//#define INDIVIDUAL_EYE_THREAD
#define ENABLE_STG
//#define __USE_AR_STG_CALIB_TEST
//#define SS_HEAD
//#define NO_DEPTHMAP

using namespace std;
using namespace cv;

#include "../kar_helpers.hpp"
#include "../ar_settings/ArSettings.h"

// This example will require several standard data-structures and algorithms:
#define _USE_MATH_DEFINES
#include <math.h>
#include <queue>
#include <unordered_set>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include <iostream>

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>

SS_Tool_Guide_Pts ss_tool_info;

int tumor_id = 0;
bool loadScrewTest2(const std::string& screwfile, const std::string& tumor_model_file, std::vector<int>& line_ids, std::vector<glm::fvec3>& guide_lines)
{
	std::ifstream infile(screwfile);
	string line;
	if (infile.is_open())
	{
		getline(infile, line);
		std::istringstream iss_num(line);

		int screwcount;
		iss_num >> screwcount;

		int _line_idx = 0;
		while (getline(infile, line))
		{
			std::istringstream iss(line);
			float a, b, c, d, e, f, g;
			if (!(iss >> a >> b >> c >> d >> e >> f)) { break; } // error

			const float line_leng = 10.f;
			glm::fvec3 p = glm::fvec3(a, b, c);
			glm::fvec3 dir = glm::normalize(glm::fvec3(d, e, f) - p);
			int line_id = 0;
			//vzm::GenerateLinesObject(__FP p, __FP (p + dir * line_leng), 1, line_id);
			line_ids.push_back(line_id);
			guide_lines.push_back(p);
			guide_lines.push_back(dir);
			_line_idx++;
		}
		infile.close();

		//vzm::GenerateCylindersObject((float*)&needles_pos[0], &needles_radii[0], (float*)&needles_clr[0], screwcount, needles_guide_id);

	}

	vzm::LoadModelFile(tumor_model_file, tumor_id);

	ss_tool_info.pos_centers_tfrm.clear();
	infile = std::ifstream(var_settings::GetDefaultFilePath() + "..\\Preset\\ss_tool_pts.txt");
	line = "";
	while (getline(infile, line))
	{
		std::istringstream iss(line);
		float a, b, c;
		if (!(iss >> a >> b >> c)) { break; } // error
		ss_tool_info.pos_centers_tfrm.push_back(glm::fvec3(a, b, c));
		// process pair (a,b)
	}
	infile.close();

	return true;
}

int main()
{

#if defined(_DEBUG) | defined(DEBUG)
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif	

	vzm::InitEngineLib();
	if (!optitrk::InitOptiTrackLib())
	{
		printf("Unable to license Motive API\n");
		return 1;
	}

	// After initial post-processing, frames will flow into this queue:
	rs2::frame_queue original_data;
	rs2::frame_queue filtered_data;
	rs2::frame_queue eye_data;

	const int eye_w = 640;
	const int eye_h = 480;
	const int ws_w = 640;
	const int ws_h = 480;
	const int stg_w = 960;
	const int stg_h = 540;
	const int rs_w = 960;
	const int rs_h = 540;

	int breast_bone_id = 0;
	string breast_bone_path = "..\\Data\\breast\\breast_bone.stl";
	vzm::LoadModelFile(breast_bone_path, breast_bone_id);

	rs_settings::InitializeRealsense(true, false, rs_w, rs_h, eye_w, eye_h);
	rs_settings::RunRsThread(original_data, filtered_data, eye_data);
	//rs2_intrinsics rgb_intrinsics, depth_intrinsics;
	//rs2_extrinsics rgb_extrinsics;
	//rs_settings::GetRsCamParams(rgb_intrinsics, depth_intrinsics, rgb_extrinsics);

	var_settings::InitializeVarSettings(1, true, "marker");
	var_settings::SetCvWindows();
	var_settings::SetPreoperations(rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);

	//optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("ss_tool_v2", 0.1f, 1);
	optitrk::SetRigidBodyEnabledbyName("ss_head", false);
	optitrk::SetRigidBodyEnabledbyName("marker", false);
	optitrk::SetRigidBodyEnabledbyName("rs_cam", true);
	optitrk::SetRigidBodyEnabledbyName("probe", true);
	optitrk::SetRigidBodyEnabledbyName("breastbody", true);
	optitrk::SetRigidBodyEnabledbyName("spine", false);
	optitrk::SetRigidBodyEnabledbyName("tool_1", false);
	optitrk::SetRigidBodyEnabledbyName("tool_2", false);
	optitrk::SetRigidBodyEnabledbyName("tool_3", false);
	optitrk::SetRigidBodyEnabledbyName("ss_tool_v1", false);
	optitrk::SetRigidBodyEnabledbyName("ss_tool_v2", false);

	std::string pin_tool_name = "ss_tool_v2";
	optitrk::SetRigidBodyEnabledbyName(pin_tool_name, true);

	int postpone = 3;
#define NUM_RBS 5
	concurrent_queue<track_info> track_que(10);
	std::atomic_bool tracker_alive{ true };
	std::thread tracker_processing_thread([&]() {
		while (tracker_alive)
		{
			Sleep(postpone);
			optitrk::UpdateFrame();

			track_info cur_trk_info;
			static string _rb_names[NUM_RBS] = { "rs_cam" , "probe", "marker" , pin_tool_name , "breastbody" };
			for (int i = 0; i < NUM_RBS; i++)
			{
				glm::fmat4x4 mat_lfrm2ws;
				bool is_detected = optitrk::GetRigidBodyLocationByName(_rb_names[i], (float*)&mat_lfrm2ws);
				cur_trk_info.SetLFrmInfo(_rb_names[i], is_detected, mat_lfrm2ws);
			}

			//cout << cur_is_rsrb_detected << ", " << cur_is_probe_detected << endl;

			optitrk::GetMarkersLocation(&cur_trk_info.mk_xyz_list, &cur_trk_info.mk_residue_list, &cur_trk_info.mk_cid_list);
			cur_trk_info.is_updated = true;
			track_que.push(cur_trk_info);
		}
		});

	GlobalInfo* _ginfo;
	var_settings::GetVarInfoPtr((void**)&_ginfo);
	GlobalInfo& ginfo = *_ginfo;

	std::vector<int> guide_line_ids;
	std::vector<glm::fvec3> guide_lines;
	loadScrewTest2(var_settings::GetDefaultFilePath() + "..\\Data\\breast\\chest_pins.txt", var_settings::GetDefaultFilePath() + "..\\Data\\breast\\tumor_pos.stl", guide_line_ids, guide_lines);
	vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.rs_scene_id, 1, tumor_id);
	vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, tumor_id);

	// params for the main thread
	int key_pressed = -1;
	bool show_apis_console = false;
	bool show_csection = false;
	bool show_mks = true;
	bool show_calib_frames = true;
	bool record_info = false;
	bool show_pc = false;
	bool show_workload = false;
	bool is_ws_pick = false;

	auto DisplayTimes = [&show_workload](const LARGE_INTEGER lIntCntStart, const string& _test)
	{
		if (!show_workload) return;
		LARGE_INTEGER lIntFreq, lIntCntEnd;
		QueryPerformanceFrequency(&lIntFreq);
		QueryPerformanceCounter(&lIntCntEnd);
		double dRunTime1 = (lIntCntEnd.QuadPart - lIntCntStart.QuadPart) / (double)(lIntFreq.QuadPart);

		cout << _test << " : " << 1. / dRunTime1 << " fps" << endl;
	};
	auto GetPerformanceFreq = []() -> LARGE_INTEGER
	{
		LARGE_INTEGER lIntCntFreq;
		QueryPerformanceCounter(&lIntCntFreq);
		return lIntCntFreq;
	};

#ifdef __RECORD_VER
	// fill record_trk_info and record_rsimg
#endif
	vzm::SetRenderTestParam("_double4_ShadingFactorsForGlobalPrimitives", glm::dvec4(0.8, 2.5, 1.0, 30.0), sizeof(glm::dvec4), 5, 1);

	vzm::SetRenderTestParam("_bool_GhostEffect", true, sizeof(bool), ginfo.rs_scene_id, 1);
	vzm::SetRenderTestParam("_bool_UseMask3DTip", true, sizeof(bool), -1, -1);
	vzm::SetRenderTestParam("_double4_MaskCenterRadius0", glm::dvec4(-100, -100, 0.1, 0.5), sizeof(glm::dvec4), -1, -1);
	vzm::SetRenderTestParam("_double3_HotspotParamsTKtKs0", glm::dvec3(0.002, 0.8, 2.0), sizeof(glm::dvec3), -1, -1);
	vzm::SetRenderTestParam("_double_InDepthVis", 0.10, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_int_OitMode", (int)0, sizeof(int), -1, -1);

	vzm::ObjStates brest_bone_states;
	*(glm::fvec4*)brest_bone_states.color = glm::fvec4(1);

	int line_guide_idx = 0;
	int operation_step = 0;
	std::string preset_path = var_settings::GetDefaultFilePath();
	ginfo.custom_pos_file_paths["ss_tool_v2"] = preset_path + "..\\Preset\\ss_tool_v2_se.txt";
	var_settings::LoadPresets();

	std::string probe_name = "probe";
	PROBE_MODE probe_mode = PROBE_MODE::DEFAULT;

	vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, breast_bone_id);
	vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, tumor_id);

	while (key_pressed != 'q' && key_pressed != 27)
	{
		LARGE_INTEGER frq_begin = GetPerformanceFreq();
		bool reset_calib = false;
		bool write_recoded_info = false;
		bool recompile_hlsl = false;
		switch (key_pressed) // http://www.asciitable.com/
		{
		case '[': postpone = max(postpone - 1, 0); cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
		case ']': postpone += 1; cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
		case 'r': recompile_hlsl = true; cout << "Recompile Shader!" << endl; break;
		case 'v': show_calib_frames = !show_calib_frames; break;
		case 'p': show_pc = !show_pc; break;
		case 'e': show_apis_console = !show_apis_console; break;
		case 'm': show_mks = !show_mks; break;
		case 's': show_csection = !show_csection; break;
		case 'x': reset_calib = true; break;
		case 'd': record_info = !record_info; break;
		case 'w': write_recoded_info = true; break;
		case 'f': show_workload = !show_workload; break;
		case 'c': is_ws_pick = !is_ws_pick; break;
		case '1': operation_step = 1; probe_name = "probe"; probe_mode = PROBE_MODE::DEFAULT;
			optitrk::SetRigidBodyEnabledbyName("probe", true);
			optitrk::SetRigidBodyEnabledbyName(pin_tool_name, false);
			break;
		case '2': operation_step = 2; probe_name = pin_tool_name; probe_mode = PROBE_MODE::ONLY_RBFRAME;
			optitrk::SetRigidBodyEnabledbyName("probe", false);
			optitrk::SetRigidBodyEnabledbyName(pin_tool_name, true); 
			break;
		case '7': operation_step = 7; probe_name = "probe"; probe_mode = PROBE_MODE::DEFAULT;
			optitrk::SetRigidBodyEnabledbyName("probe", true);
			optitrk::SetRigidBodyEnabledbyName(pin_tool_name, true);
			ginfo.dst_tool_name = pin_tool_name;
			ginfo.src_tool_name = "probe";
			break;
		case ',': line_guide_idx = max(line_guide_idx - 1, 0); break;
		case '.': line_guide_idx = min(line_guide_idx + 1, (int)guide_line_ids.size() - 1); break;
		}
		vzm::SetRenderTestParam("_bool_ReloadHLSLObjFiles", recompile_hlsl, sizeof(bool), -1, -1);
		vzm::SetRenderTestParam("_bool_PrintOutRoutineObjs", show_apis_console, sizeof(bool), -1, -1);
		vzm::DisplayConsoleMessages(show_apis_console);

		if (reset_calib) var_settings::ResetCalib();

		if (write_recoded_info) var_settings::StoreRecordInfo();

		// Fetch the latest available post-processed frameset
		//static rs2::frameset frameset0, frameset1;
		rs2::frameset current_frameset;
		original_data.poll_for_frame(&current_frameset);
		rs2::frame current_filtered_frame;
		filtered_data.poll_for_frame(&current_filtered_frame);

		track_info trk_info;
		track_que.wait_and_pop(trk_info);

		if (trk_info.is_updated && current_frameset)
		{
			DisplayTimes(frq_begin, "device_stream_load");

			var_settings::UpdateTrackInfo(&trk_info, probe_name, probe_mode);

			auto current_color_frame = current_frameset.get_color_frame();
			//auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

			if (record_info) var_settings::RecordInfo(key_pressed, current_color_frame.get_data());

			//const int w = color.as<rs2::video_frame>().get_width(); // rs_w
			//const int h = color.as<rs2::video_frame>().get_height(); // rs_h

			Mat image_rs(Size(rs_w, rs_h), CV_8UC3, (void*)current_color_frame.get_data(), Mat::AUTO_STEP), image_rs_bgr;
			cvtColor(image_rs, image_rs_bgr, COLOR_BGR2RGB);

			var_settings::SetTcCalibMkPoints();
			var_settings::SetMkSpheres(show_mks, is_ws_pick);

			var_settings::TryCalibrationTC(image_rs_bgr);
			var_settings::TryCalibrationSTG();

			var_settings::SetCalibFrames(show_calib_frames);

			rs2::depth_frame depth_frame = current_filtered_frame;
			var_settings::SetDepthMapPC(show_pc, depth_frame, current_color_frame);

			var_settings::SetTargetModelAssets("breastbody", __FP guide_lines[0], guide_lines.size() / 2, line_guide_idx);

			SetCustomTools(ginfo.dst_tool_name, ONLY_RBFRAME, ginfo, glm::fvec3(1, 1, 0), operation_step >= 7);

			var_settings::SetSectionalImageAssets(ginfo.is_modelaligned, __FP ginfo.pos_probe_pin, __FP(ginfo.pos_probe_pin + ginfo.dir_probe_se * 0.2f));

			// tumor vis.
			{
				if (ginfo.is_modelaligned)
				{
					glm::fmat4x4 mat_matchmodelfrm2ws;
					if (ginfo.otrk_data.trk_info.GetLFrmInfo("breastbody", mat_matchmodelfrm2ws))
					{
						vzm::ObjStates model_ws_obj_state;
						vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_obj_state);
						__cm4__ model_ws_obj_state.os2ws = mat_matchmodelfrm2ws * ginfo.mat_os2matchmodefrm;

						__cv4__ model_ws_obj_state.color = glm::fvec4(1, 0, 0, 1);
						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tumor_id, model_ws_obj_state);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tumor_id, model_ws_obj_state);
						vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tumor_id, model_ws_obj_state);
						vzm::ReplaceOrAddSceneObject(ginfo.csection_scene_id, tumor_id, model_ws_obj_state);

						__cm4__ brest_bone_states.os2ws = __cm4__ model_ws_obj_state.os2ws;
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, breast_bone_id, brest_bone_states);
					}
				}
			}

			var_settings::RenderAndShowWindows(show_workload, image_rs_bgr);
		}

		key_pressed = cv::waitKey(1);
	}

	var_settings::DeinitializeVarSettings();
	// Signal threads to finish and wait until they do
	rs_settings::FinishRsThreads();
	rs_settings::DeinitializeRealsense();
	tracker_alive = false;
	tracker_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
	vzm::DeinitEngineLib();

	return 0;
}