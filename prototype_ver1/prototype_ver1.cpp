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

int cancer_id = 0;
int needles_guide_id = 0;
bool loadScrewInha(std::string screwfile)
{
	std::ifstream screwinfile(screwfile, std::ios_base::binary);

	if (!screwinfile.good())
		return false;

	int screwcount = 0;
	//screwinfile >> screwcount;
	screwinfile.read((char*)(&screwcount), 4);

	glm::vec3 startpos, endpos;
	float screwdiameter;
	/*
	screw number
	1st screw start position	1st screw end position	1st screw diameter
	2nd screw end position		2nd screw end position	2nd screw diameter
	.
	.
	.
	n-th screw start position	n-th screw end position	n-th screw diameter
	*/
	vector<float> needles_radii(screwcount);
	vector<glm::fvec3> needles_pos(screwcount * 2);
	vector<glm::fvec3> needles_clr(screwcount);
	for (int i = 0; i < screwcount; i++)
	{
		screwinfile.read((char*)(&startpos), sizeof(glm::vec3));
		screwinfile.read((char*)(&endpos), sizeof(glm::vec3));
		screwinfile.read((char*)(&screwdiameter), sizeof(float));

		//glm::vec3 direction = endpos - startpos;
		//float length = glm::length(direction);

		// pos »óÀÇ tr

		needles_pos[2 * i + 0] = startpos;
		needles_pos[2 * i + 1] = endpos;
		needles_clr[i] = glm::fvec3(0.2, 0.8, 1);
		needles_radii[i] = screwdiameter / 2.0;
		//this->PreloadedScrew_target->push_back(ScrewModelInfo(startpos, direction, length, screwdiameter / 2.0));
	}

	vzm::GenerateCylindersObject((float*)&needles_pos[0], &needles_radii[0], (float*)&needles_clr[0], screwcount, needles_guide_id);

	return true;
}

bool loadScrewTest(std::string screwfile)
{

	std::ifstream infile(screwfile);
	string line;
	if (infile.is_open())
	{
		getline(infile, line);
		std::istringstream iss_num(line);

		int screwcount;
		iss_num >> screwcount;
		
		vector<float> needles_radii(screwcount);
		vector<glm::fvec3> needles_pos(screwcount * 2);
		vector<glm::fvec3> needles_clr(screwcount);

		int _line_idx = 0;
		while (getline(infile, line))
		{
			std::istringstream iss(line);
			float a, b, c, d, e, f, g;
			if (!(iss >> a >> b >> c >> d >> e >> f >> g)) { break; } // error

			needles_pos[2 * _line_idx + 0] = glm::fvec3(a, b, c);
			needles_pos[2 * _line_idx + 1] = glm::fvec3(d, e, f);
			needles_clr[_line_idx] = glm::fvec3(0.2, 0.8, 1);
			needles_radii[_line_idx] = g / 2.f * 2.f;
			_line_idx++;
		}
		infile.close();

		vzm::GenerateCylindersObject((float*)&needles_pos[0], &needles_radii[0], (float*)&needles_clr[0], screwcount, needles_guide_id);
	}

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
	const int stg_w = 640;
	const int stg_h = 480;
	const int rs_w = 960;
	const int rs_h = 540;

	rs_settings::InitializeRealsense(true, false, rs_w, rs_h, eye_w, eye_h);
	rs_settings::RunRsThread(original_data, filtered_data, eye_data);
	//rs2_intrinsics rgb_intrinsics, depth_intrinsics;
	//rs2_extrinsics rgb_extrinsics;
	//rs_settings::GetRsCamParams(rgb_intrinsics, depth_intrinsics, rgb_extrinsics);

	var_settings::InitializeVarSettings(0);
	var_settings::SetCvWindows();
	var_settings::SetPreoperations(rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);

	loadScrewTest(var_settings::GetDefaultFilePath() + "..\\Data\\breast\\chest_pins.txt");

	optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("ss_tool_v1", 0.1f, 1);
	int postpone = 3;
	concurrent_queue<track_info> track_que(10);
	std::atomic_bool tracker_alive{ true };
	std::thread tracker_processing_thread([&]() {
		while (tracker_alive)
		{
			Sleep(postpone);
			optitrk::UpdateFrame();

			track_info cur_trk_info;
			static string _rb_names[5] = { "rs_cam" , "probe" , "ss_tool_v1" , "ss_head" , "breastbody" };
			for (int i = 0; i < 5; i++)
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

#ifdef EYE_VIS_RS
	string window_name_eye_view = "EYE VIEW";
	cv::namedWindow(g_info.window_name_eye_view, WINDOW_NORMAL);
	cv::moveWindow(g_info.window_name_eye_view, 2560, 0);
#endif

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
	var_settings::LoadPresets();
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
			// RsMouseMode
		//case '1': g_info.touch_mode = RsTouchMode::None; break; 
		//case '2': g_info.touch_mode = RsTouchMode::Pick; break; 
		//case '3': g_info.touch_mode = RsTouchMode::Calib_TC; break; 
		//case '4': g_info.touch_mode = RsTouchMode::PIN_ORIENTATION; break; 
		//case '5': g_info.touch_mode = RsTouchMode::Calib_STG; break; 
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

			var_settings::UpdateTrackInfo(&trk_info);

			auto current_color_frame = current_frameset.get_color_frame();
			//auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

			if (record_info) var_settings::RecordInfo(key_pressed, current_color_frame.get_data());

			//const int w = color.as<rs2::video_frame>().get_width(); // rs_w
			//const int h = color.as<rs2::video_frame>().get_height(); // rs_h

			Mat image_rs(Size(rs_w, rs_h), CV_8UC3, (void*)current_color_frame.get_data(), Mat::AUTO_STEP), image_rs_bgr;
			cvtColor(image_rs, image_rs_bgr, COLOR_BGR2RGB);

			var_settings::SetTcCalibMkPoints(is_ws_pick);
			var_settings::SetMkSpheres(show_mks, is_ws_pick);

			var_settings::TryCalibrationTC(image_rs_bgr);
			var_settings::TryCalibrationSTG();

			var_settings::SetCalibFrames(show_calib_frames);

			rs2::depth_frame depth_frame = current_filtered_frame;
			var_settings::SetDepthMapPC(show_pc, depth_frame, current_color_frame);

			var_settings::SetTargetModelAssets("ss_head"); // "breastbody" "ss_head"

			// SS tool custom vis.
			{
				GlobalInfo ginfo;
				var_settings::GetVarInfo(&ginfo);

				glm::fmat4x4 mat_sstool2ws;
				bool is_sstool_detected = trk_info.GetLFrmInfo("ss_tool_v1", mat_sstool2ws);
				if (is_sstool_detected)
				{
					if (ss_tool_info.pos_centers_tfrm.size() > 0)
					{
						vector<glm::fvec4> sphers_xyzr;
						vector<glm::fvec3> sphers_rgb;
						glm::fvec3 colors[3] = { glm::fvec3(1, 0, 0) , glm::fvec3(0, 1, 0) , glm::fvec3(0, 0, 1) };
						for (int i = 0; i < (int)ss_tool_info.pos_centers_tfrm.size(); i++)
						{
							sphers_xyzr.push_back(glm::fvec4(ss_tool_info.pos_centers_tfrm[i], 0.005));
							sphers_rgb.push_back(colors[i % 3]);
						}

						vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], ss_tool_info.pos_centers_tfrm.size(), ss_tool_info.ss_tool_guide_points_id);

						vzm::ObjStates cobjstate;
						*(glm::fmat4x4*) cobjstate.os2ws = mat_sstool2ws;
						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
						vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
					}
				}
			}

			var_settings::RenderAndShowWindows(show_workload, image_rs_bgr);
		}

#ifdef EYE_VIS_RS
		rs2::frameset eye_current_frameset;
		eye_data.poll_for_frame(&eye_current_frameset);
		if (eye_current_frameset)
		{
			auto color = eye_current_frameset.get_color_frame();
			const int w = color.as<rs2::video_frame>().get_width();
			const int h = color.as<rs2::video_frame>().get_height();

			Mat eye_image_rs(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
			Mat eye_imagebgr;

			cvtColor(eye_image_rs, eye_imagebgr, COLOR_BGR2RGB);
			imshow(window_name_eye_view, eye_imagebgr);
		}
#endif
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