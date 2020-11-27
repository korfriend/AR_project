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

bool loadScrewGuideLines(const std::string& screwfile, std::vector<int>& line_ids, std::vector<glm::fvec3>& guide_lines)
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

	vzm::SetRenderTestParam("_double4_ShadingFactorsForGlobalPrimitives", glm::dvec4(0.8, 2.5, 1.0, 30.0), sizeof(glm::dvec4), 5, 1);

	GlobalInfo* _ginfo;
	var_settings::GetVarInfoPtr((void**)&_ginfo);
	GlobalInfo& ginfo = *_ginfo;
	vzm::SetRenderTestParam("_bool_GhostEffect", true, sizeof(bool), ginfo.rs_scene_id, 1);
	vzm::SetRenderTestParam("_bool_UseMask3DTip", true, sizeof(bool), -1, -1);
	vzm::SetRenderTestParam("_double4_MaskCenterRadius0", glm::dvec4(-100, -100, 50, 0.5), sizeof(glm::dvec4), -1, -1);
	vzm::SetRenderTestParam("_double3_HotspotParamsTKtKs0", glm::dvec3(1, 0.5, 1.5), sizeof(glm::dvec3), -1, -1);
	vzm::SetRenderTestParam("_double_InDepthVis", 0.01, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), 0, 0, ginfo.model_ws_obj_id);
	vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), 0, 0, ginfo.brain_ws_obj_id);


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

	var_settings::InitializeVarSettings(2, "tool_3");
	var_settings::SetCvWindows();
	var_settings::SetPreoperations(rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);

	std::vector<int> guide_line_ids;
	std::vector<glm::fvec3> guide_lines;
	loadScrewGuideLines("C:\\Users\\User\\source\\repos\\korfriend\\LargeData\\spine_pins.txt", guide_line_ids, guide_lines);

	//optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("tool_1", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("tool_2", 0.1f, 1);
	//optitrk::SetRigidBodyPropertyByName("tool_3", 0.1f, 1);
	optitrk::SetRigidBodyEnabledbyName("ss_head", false);
	optitrk::SetRigidBodyEnabledbyName("marker", false);
	optitrk::SetRigidBodyEnabledbyName("rs_cam", true);
	optitrk::SetRigidBodyEnabledbyName("probe", true);
	optitrk::SetRigidBodyEnabledbyName("breastbody", false);
	optitrk::SetRigidBodyEnabledbyName("spine", true);
	optitrk::SetRigidBodyEnabledbyName("tool_1", false);
	optitrk::SetRigidBodyEnabledbyName("tool_2", false);
	optitrk::SetRigidBodyEnabledbyName("tool_3", false);
	optitrk::SetRigidBodyEnabledbyName("ss_tool_v1", false);
	optitrk::SetRigidBodyEnabledbyName("ss_tool_v2", false);
	int postpone = 3;
	concurrent_queue<track_info> track_que(10);
	std::atomic_bool tracker_alive{ true };
	int operation_step = 0;
#define NUM_RBS 6
	std::thread tracker_processing_thread([&]() {
		while (tracker_alive)
		{
			Sleep(postpone);
			optitrk::UpdateFrame();

			switch (operation_step)
			{
			case 1:
				optitrk::SetRigidBodyEnabledbyName("tool_1", true);
				optitrk::SetRigidBodyEnabledbyName("tool_2", false);
				optitrk::SetRigidBodyEnabledbyName("tool_3", false);
				optitrk::SetRigidBodyEnabledbyName("probe", false);
				break;
			case 2:
				optitrk::SetRigidBodyEnabledbyName("tool_1", false);
				optitrk::SetRigidBodyEnabledbyName("tool_2", true);
				optitrk::SetRigidBodyEnabledbyName("tool_3", false);
				optitrk::SetRigidBodyEnabledbyName("probe", false);
				break;
			case 3:
				optitrk::SetRigidBodyEnabledbyName("tool_1", false);
				optitrk::SetRigidBodyEnabledbyName("tool_2", false);
				optitrk::SetRigidBodyEnabledbyName("tool_3", true);
				optitrk::SetRigidBodyEnabledbyName("probe", false);
				break;
			case 7:
				optitrk::SetRigidBodyEnabledbyName("tool_1", true);
				optitrk::SetRigidBodyEnabledbyName("tool_2", true);
				optitrk::SetRigidBodyEnabledbyName("tool_3", false);
				optitrk::SetRigidBodyEnabledbyName("probe", false);
				break;
			case 8:
				optitrk::SetRigidBodyEnabledbyName("tool_1", true);
				optitrk::SetRigidBodyEnabledbyName("tool_2", false);
				optitrk::SetRigidBodyEnabledbyName("tool_3", true);
				optitrk::SetRigidBodyEnabledbyName("probe", false);
				break;
			default:
				optitrk::SetRigidBodyEnabledbyName("tool_1", false);
				optitrk::SetRigidBodyEnabledbyName("tool_2", false);
				optitrk::SetRigidBodyEnabledbyName("tool_3", false);
				optitrk::SetRigidBodyEnabledbyName("probe", true);
				break;
			}
			track_info cur_trk_info;
			static string _rb_names[NUM_RBS] = { "rs_cam" , "probe" , "spine" , "tool_1" , "tool_2", "tool_3" };
			for (int i = 0; i < NUM_RBS; i++)
			{
				glm::fmat4x4 mat_lfrm2ws;
				bool is_detected = optitrk::GetRigidBodyLocationByName(_rb_names[i], (float*)&mat_lfrm2ws);
				cur_trk_info.SetLFrmInfo(_rb_names[i], is_detected, mat_lfrm2ws);

				//if (i == 3)
				//	cout << i << " : " << is_detected << endl;
				//if (i == 4)
				//	cout << i << " : " << is_detected << endl;
				//if (i == 5)
				//	cout << i << " : " << is_detected << endl;
			}

			//cout << cur_is_rsrb_detected << ", " << cur_is_probe_detected << endl;

			optitrk::GetMarkersLocation(&cur_trk_info.mk_xyz_list, &cur_trk_info.mk_residue_list, &cur_trk_info.mk_cid_list);
			cur_trk_info.is_updated = true;
			track_que.push(cur_trk_info);
		}
		});

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

	vzm::DisplayConsoleMessages(false);

	int line_guide_idx = 0;
	var_settings::LoadPresets();

	{
		std::string preset_path = var_settings::GetDefaultFilePath();
		auto load_preset_tools = [&preset_path, &ginfo](const int tool_idx)
		{
			std::ifstream infile = std::ifstream(preset_path + "..\\Preset\\tool_" + to_string(tool_idx) + (tool_idx == 3 ? "_se.txt": "_end.txt"));
			string line;
			if (infile.is_open())
			{
				vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_" + to_string(tool_idx)];
				while (getline(infile, line))
				{
					std::istringstream iss(line);
					float a, b, c;
					if (!(iss >> a >> b >> c)) { break; } // error
					custom_pos_list.push_back(Point3f(a, b, c));
					// process pair (a,b)
				}
				infile.close();
			}
		};
		load_preset_tools(1);
		load_preset_tools(2);
		load_preset_tools(3);
	}


	while (key_pressed != 'q' && key_pressed != 27)
	{
		LARGE_INTEGER frq_begin = GetPerformanceFreq();
		bool reset_calib = false;
		bool write_recoded_info = false;
		bool recompile_hlsl = false;
		bool insert = false;
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
		case '`': operation_step = 0; break;
		case '1': operation_step = 1; break;
		case '2': operation_step = 2; break;
		case '3': operation_step = 3; break;
		case '7': 
		case '8':
		case '9': operation_step = 7; break; // temp setting step...
		case '-':
		case '*':
		case '=': operation_step = 8; break; // temp setting step...
		case ',': line_guide_idx = max(line_guide_idx - 1, 0); break;
		case '.': line_guide_idx = min(line_guide_idx + 1, (int)guide_line_ids.size() - 1); break;
		//case 'i': insert = true; break;
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

			string probe_name = "probe";
			if (operation_step == 1) probe_name = "tool_1";
			else if (operation_step == 2) probe_name = "tool_2";
			else if (operation_step == 3) probe_name = "tool_3";
			var_settings::UpdateTrackInfo(&trk_info, probe_name);

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

			var_settings::SetTargetModelAssets("spine"); 

			var_settings::SetSectionalImageAssets(false, NULL, NULL);

			// new features
			auto register_tool_obj = [&ginfo](const glm::fvec3& pos_tool_tip, const glm::fvec3& tool_dir,
				const float tool_length, const float tool_r,
				const glm::fvec3& cyl_rgb, int& tool_id, int& tool_tip_id)
			{
				glm::fvec3 cyl_p[2] = { pos_tool_tip, pos_tool_tip + tool_dir * tool_length };
				float cyl_r = tool_r;

				vzm::ObjStates tool_line, tool_tip;
				vzm::GenerateCylindersObject((float*)cyl_p, &cyl_r, __FP cyl_rgb, 1, tool_id);
				glm::fvec4 tip_sphere_xyzr = glm::fvec4(pos_tool_tip, cyl_r);
				vzm::GenerateSpheresObject(__FP tip_sphere_xyzr, NULL, 1, tool_tip_id);
				vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tool_id, tool_line);
				vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tool_id, tool_line);
				vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tool_id, tool_line);
				__cv4__ tool_tip.color = glm::fvec4(1, 0, 0, 1);
				vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tool_tip_id, tool_tip);
				vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tool_tip_id, tool_tip);
				vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tool_tip_id, tool_tip);
			};

			static int tool_screw_id = 0;
			static int tool_id = 0, tool_tip_id = 0;
			static int closest_dist_line_id = 0, closest_dist_text_id = 0;
			static int angle_id = 0, angle_text_id = 0, angle_text_id_stg = 0;
			static int tools_ids[3] = { 0, 0, 0 };
			static int tool_tips_ids[3] = { 0, 0, 0 };
			static int tool_3_se_spheres_id = 0;
			int scenario_3_objs[] = { tool_screw_id , tool_id , tool_tip_id , closest_dist_line_id , closest_dist_text_id ,
				angle_id , angle_text_id , angle_text_id_stg, tools_ids[0], tools_ids[1], tools_ids[2], 
				tool_tips_ids[0] , tool_tips_ids[1], tool_tips_ids[2], tool_3_se_spheres_id };
			vzm::ObjStates scenario_3_obj_state;
			scenario_3_obj_state.is_visible = false;
			for (int i = 0; i < (int)(sizeof(scenario_3_objs) / sizeof(int)); i++)
			{
				int obj_id = scenario_3_objs[i];
				vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, obj_id, scenario_3_obj_state);
				vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, obj_id, scenario_3_obj_state);
				vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, obj_id, scenario_3_obj_state);
			}
			for (int i = 0; i < guide_line_ids.size(); i++)
			{
				int obj_id = guide_line_ids[i];
				vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, obj_id, scenario_3_obj_state);
				vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, obj_id, scenario_3_obj_state);
				vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, obj_id, scenario_3_obj_state);
			}

			if (operation_step == 7 || operation_step == 8)
			{
				bool is_tool_1_tracked = false, is_tool_2_tracked = false, is_tool_3_tracked = false;
				glm::fmat4x4 mat_l1frm2ws, mat_l2frm2ws, mat_l3frm2ws;
				is_tool_1_tracked = trk_info.GetLFrmInfo("tool_1", mat_l1frm2ws);
				is_tool_2_tracked = trk_info.GetLFrmInfo("tool_2", mat_l2frm2ws);
				is_tool_3_tracked = trk_info.GetLFrmInfo("tool_3", mat_l3frm2ws);
				if (is_tool_1_tracked && is_tool_2_tracked)
				{
					glm::fvec3 pos_tool_end_lfrm;
					if (key_pressed == '8' || key_pressed == '9')
					{
						std::string file_path = var_settings::GetDefaultFilePath();
						if (key_pressed == '8') // set end of tool_1
						{
							glm::fvec3 pos_tool_tip = tr_pt(mat_l2frm2ws, glm::fvec3(0));
							glm::fmat4x4 mat_ws2l1frm = glm::inverse(mat_l1frm2ws);
							pos_tool_end_lfrm = tr_pt(mat_ws2l1frm, pos_tool_tip);
							file_path += "..\\Preset\\tool_1_end.txt";
							vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_1"];
							custom_pos_list.push_back(Point3f(pos_tool_end_lfrm.x, pos_tool_end_lfrm.y, pos_tool_end_lfrm.z));
						}
						else if (key_pressed == '9') // set end of tool_2
						{
							glm::fvec3 pos_tool_tip = tr_pt(mat_l1frm2ws, glm::fvec3(0));
							glm::fmat4x4 mat_ws2l2frm = glm::inverse(mat_l2frm2ws);
							pos_tool_end_lfrm = tr_pt(mat_ws2l2frm, pos_tool_tip);
							file_path += "..\\Preset\\tool_2_end.txt";
							vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_2"];
							custom_pos_list.push_back(Point3f(pos_tool_end_lfrm.x, pos_tool_end_lfrm.y, pos_tool_end_lfrm.z));
						}

						ofstream outfile(file_path);
						if (outfile.is_open())
						{
							outfile.clear();
							string line = to_string(pos_tool_end_lfrm.x) + " " +
								to_string(pos_tool_end_lfrm.y) + " " +
								to_string(pos_tool_end_lfrm.z);
							outfile << line << endl;
						}
						outfile.close();
					}
				}
				else if (is_tool_1_tracked && is_tool_3_tracked)
				{
					glm::fvec3 pos_tool_end_lfrm;
					vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_3"];
					if (key_pressed == '*')
					{
						custom_pos_list.clear();
					}
					else if (key_pressed == '=')
					{
						glm::fvec3 pos_tool_tip = tr_pt(mat_l1frm2ws, glm::fvec3(0));
						glm::fmat4x4 mat_ws2l3frm = glm::inverse(mat_l3frm2ws);
						pos_tool_end_lfrm = tr_pt(mat_ws2l3frm, pos_tool_tip);
						custom_pos_list.push_back(Point3f(pos_tool_end_lfrm.x, pos_tool_end_lfrm.y, pos_tool_end_lfrm.z));
						
						std::string file_path = var_settings::GetDefaultFilePath() + "..\\Preset\\tool_3_se.txt";
						//std::string file_path = "D:\\tool_3_end.txt";
						ofstream outfile(file_path);
						if (outfile.is_open())
						{
							outfile.clear();
							for (int i = 0; i < custom_pos_list.size(); i++)
							{
								string line = to_string(custom_pos_list[i].x) + " " +
									to_string(custom_pos_list[i].y) + " " +
									to_string(custom_pos_list[i].z);
								outfile << line << endl;
							}
						}
						outfile.close();
					}

					if (custom_pos_list.size() > 0)
					{
						vector<glm::fvec4> sphers_xyzr;
						vector<glm::fvec3> sphers_rgb;
						for (int i = 0; i < custom_pos_list.size(); i++)
						{
							glm::fvec3 pt = tr_pt(mat_l3frm2ws, *(glm::fvec3*)&custom_pos_list[i]);
							sphers_xyzr.push_back(glm::fvec4(pt, 0.005));
							sphers_rgb.push_back(glm::fvec3(1, 0, 0));
						}
						vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], custom_pos_list.size(), tool_3_se_spheres_id);
						vzm::ObjStates tool_3_se_spheres_state;
						vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tool_3_se_spheres_id, tool_3_se_spheres_state);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tool_3_se_spheres_id, tool_3_se_spheres_state);
						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tool_3_se_spheres_id, tool_3_se_spheres_state);
					}
				}

				auto register_tool_test = [&ginfo, &register_tool_obj](const bool is_tool_tracked, const glm::fmat4x4& mat_lfrm2ws, 
					const int tool_idx, const glm::fvec3& tool_color)
				{
					if (is_tool_tracked)
					{
						glm::fvec3 pos_tool_tip = tr_pt(mat_lfrm2ws, glm::fvec3(0));
						glm::fvec3 tool_dir = glm::normalize(tr_vec(mat_lfrm2ws, glm::fvec3(0, 0, 1)));
						vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_" + to_string(tool_idx)];
						if (custom_pos_list.size() > 0)
						{
							if (tool_idx == 3 && custom_pos_list.size() >= 2)
							{
								glm::fvec3 pos_s = *(glm::fvec3*)&custom_pos_list[0];
								pos_tool_tip = tr_pt(mat_lfrm2ws, pos_s);
								glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[1];
								tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
							}
							else
							{
								glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[0];
								tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
							}
						}
						register_tool_obj(pos_tool_tip, tool_dir, 0.2f, 0.002f, tool_color, tools_ids[tool_idx - 1], tool_tips_ids[tool_idx - 1]);
					}
				};
				register_tool_test(is_tool_1_tracked, mat_l1frm2ws, 1, glm::fvec3(1, 1, 0));
				register_tool_test(is_tool_2_tracked, mat_l2frm2ws, 2, glm::fvec3(0, 1, 1));
				register_tool_test(is_tool_3_tracked, mat_l3frm2ws, 3, glm::fvec3(1, 0, 1));
			}
			else
			{
				// totl 1, 2, and 3 are exclusive!!
				bool is_tool_tracked = false;
				glm::fmat4x4 mat_lfrm2ws;
				for (int i = 1; i <= 3; i++)
				{
					is_tool_tracked = trk_info.GetLFrmInfo("tool_" + to_string(i), mat_lfrm2ws);
					if (is_tool_tracked) break;
				}
				if (is_tool_tracked)
				{
					glm::fvec3 pos_tool_tip = tr_pt(mat_lfrm2ws, glm::fvec3(0));
					glm::fvec3 tool_dir = glm::normalize(tr_vec(mat_lfrm2ws, glm::fvec3(0, 0, 1)));
					if (operation_step == 1)
					{
						vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_1"];
						if (custom_pos_list.size() > 0)
						{
							glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[0];
							tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
						}
					}
					else if (operation_step == 2)
					{
						vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map["tool_2"];
						if (custom_pos_list.size() > 0)
						{
							glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[0];
							tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
						}
					}
					else if (operation_step == 3)
					{
						vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map[ginfo.dst_tool_se_name];
						if (custom_pos_list.size() >= 2)
						{
							// use preset pos (wrt mat_lfrm2ws) //
							glm::fvec3 pos_s = *(glm::fvec3*)&custom_pos_list[0];
							glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[1];
							pos_tool_tip = tr_pt(mat_lfrm2ws, pos_s);
							tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
						}

						// 0.04
						const float screw_length = 0.04f;
						glm::fvec3 cyl_p[2] = { pos_tool_tip, pos_tool_tip - tool_dir * screw_length };
						glm::fvec3 cyl_rgb = glm::fvec3(1, 0, 1);
						float cyl_r = 0.003f;
						vzm::ObjStates tool_screw;
						vzm::GenerateCylindersObject((float*)cyl_p, &cyl_r, __FP cyl_rgb, 1, tool_screw_id);
						vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tool_screw_id, tool_screw);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tool_screw_id, tool_screw);
						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tool_screw_id, tool_screw);
					}

					// show tool line
					//register_tool_obj(pos_tool_tip, tool_dir, 0.2f, 0.002f, glm::fvec3(1, 1, 0), tool_id, tool_tip_id);

					if (ginfo.is_modelaligned)
					{
						glm::fmat4x4 mat_matchmodelfrm2ws;
						if (ginfo.otrk_data.trk_info.GetLFrmInfo("spine", mat_matchmodelfrm2ws))
						{
							vzm::ObjStates model_ws_obj_state;
							vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_obj_state);
							glm::fmat4x4 tr = mat_matchmodelfrm2ws * ginfo.mat_os2matchmodefrm;

							glm::fvec3 pos_guide_line = tr_pt(tr, guide_lines[2 * line_guide_idx + 0]);
							glm::fvec3 dir_guide_line = glm::normalize(tr_vec(tr, guide_lines[2 * line_guide_idx + 1]));

							glm::fvec3 closetPoint;
							ComputeClosestPointBetweenLineAndPoint(pos_guide_line, dir_guide_line, pos_tool_tip, closetPoint);

							// guide line
							glm::fvec3 line_pos[2] = { pos_guide_line, pos_guide_line + dir_guide_line * 10.f };
							int& guide_line_id = guide_line_ids[line_guide_idx];
							vzm::GenerateLinesObject(__FP line_pos[0], NULL, 1, guide_line_id);
							vzm::ObjStates line_state;
							line_state.line_thickness = 5;
							vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, guide_line_id, line_state);
							vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, guide_line_id, line_state);
							vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, guide_line_id, line_state);
							SetDashEffectInRendering(ginfo.stg_scene_id, 1, guide_line_id, 0.01, false);
							SetDashEffectInRendering(ginfo.rs_scene_id, 1, guide_line_id, 0.01, false);
							SetDashEffectInRendering(ginfo.ws_scene_id, 1, guide_line_id, 0.01, false);

							// show dist line
							MakeDistanceLine(-1, pos_tool_tip, closetPoint, 0.05, closest_dist_line_id, closest_dist_text_id);
							vzm::ObjStates closest_dist_line_state;
							closest_dist_line_state.line_thickness = 5;
							vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, closest_dist_line_id, closest_dist_line_state);
							vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, closest_dist_line_id, closest_dist_line_state);
							vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, closest_dist_line_id, closest_dist_line_state);
							SetDashEffectInRendering(ginfo.stg_scene_id, 1, closest_dist_line_id, 0.01, true);
							SetDashEffectInRendering(ginfo.rs_scene_id, 1, closest_dist_line_id, 0.01, true);
							SetDashEffectInRendering(ginfo.ws_scene_id, 1, closest_dist_line_id, 0.01, true);
							// show angle
							MakeAngle2(tool_dir, dir_guide_line, closetPoint, 0.05, 0.1, angle_id, ginfo.rs_scene_id, angle_text_id, ginfo.stg_scene_id, angle_text_id_stg);
							vzm::ObjStates angle_state;
							angle_state.color[3] = 0.7;
							vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, angle_id, angle_state);
							vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, angle_text_id_stg, angle_state);
							vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, angle_id, angle_state);
							vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, angle_text_id, angle_state);
							vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, angle_id, angle_state);
							vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, angle_text_id, angle_state);
							// show MPR
							var_settings::SetSectionalImageAssets(true, __FP pos_tool_tip, __FP(pos_tool_tip + tool_dir * 20.f));
						}
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