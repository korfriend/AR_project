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
			if (!(iss >> a >> b >> c >> d >> e >> f >> g)) { break; } // error

			const float line_leng = 10.f;
			glm::fvec3 p = glm::fvec3(a, b, c);
			glm::fvec3 dir = glm::normalize(glm::fvec3(d, e, f) - p);
			int line_id = 0;
			vzm::GenerateLinesObject(__FP p, __FP (p + dir * line_leng), 1, line_id);
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

	vzm::SetRenderTestParam("_bool_UseSpinLock", true, sizeof(bool), -1, -1);
	vzm::SetRenderTestParam("_double_CopVZThickness", 0.002, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_double_AbsCopVZThickness", 0.2, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_double_VZThickness", 0.0, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_double_MergingBeta", 0.5, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_double_RobustRatio", 0.5, sizeof(double), -1, -1);

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
	loadScrewGuideLines(var_settings::GetDefaultFilePath() + "..\\Data\\breast\\spine_pins.txt", guide_line_ids, guide_lines);

	optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("tool_1", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("tool_2", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("tool_3", 0.1f, 1);
	optitrk::SetRigidBodyEnabledbyName("tool_1", false);
	optitrk::SetRigidBodyEnabledbyName("tool_2", false);
	optitrk::SetRigidBodyEnabledbyName("tool_3", false);
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
	int line_guide_idx = 0;
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
		case '`':
		case '0': operation_step = 0; break;
		case '1': operation_step = 1; break;
		case '2': operation_step = 2; break;
		case '3': operation_step = 3; break;
		case '7': operation_step = 7; break; // temp setting step...
		case ',': line_guide_idx = max(--line_guide_idx, 0); break;
		case '.': line_guide_idx = max(++line_guide_idx, (int)guide_line_ids.size() - 1); break;
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

			var_settings::SetTargetModelAssets("spine"); 

			GlobalInfo ginfo;
			var_settings::GetVarInfo(&ginfo);
			var_settings::SetSectionalImageAssets(false, NULL, NULL);
			// new features
			if(ginfo.is_modelaligned)
			{
				int guide_line_id = guide_line_ids[line_guide_idx];
				vzm::ObjStates line_state;
				line_state.line_thickness = 5;
				vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, guide_line_id, line_state);
				vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, guide_line_id, line_state);
				vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, guide_line_id, line_state);
				SetDashEffectInRendering(ginfo.stg_scene_id, 1, guide_line_id, 0.01, false);
				SetDashEffectInRendering(ginfo.rs_scene_id, 1, guide_line_id, 0.01, false);
				SetDashEffectInRendering(ginfo.ws_scene_id, 1, guide_line_id, 0.01, false);

				bool is_tool_tracked = false;
				glm::fmat4x4 mat_lfrm2ws;
				for (int i = 0; i < 3; i++)
				{
					is_tool_tracked = trk_info.GetLFrmInfo("tool_" + to_string(i), mat_lfrm2ws);
					if (is_tool_tracked) break;
				}
				if (is_tool_tracked)
				{
					glm::fvec3 pos_tool_tip = tr_pt(mat_lfrm2ws, glm::fvec3(0));
					glm::fvec3 tool_dir = glm::normalize(tr_vec(mat_lfrm2ws, glm::fvec3(0, 0, 1)));
					if (operation_step == 3)
					{
						vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map[ginfo.dst_tool_se_name];
						if (custom_pos_list.size() >= 2)
						{
							// use preset pos (wrt mat_lfrm2ws) //
							glm::fvec3 pos_s = *(glm::fvec3*)&custom_pos_list[0];
							glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[1];
							pos_tool_tip = tr_pt(ginfo.mat_probe2ws, pos_s);
							tool_dir = glm::normalize(tr_pt(ginfo.mat_probe2ws, pos_e) - pos_tool_tip);
						}

						// 0.04
						const float screw_length = 0.04f;
						static int tool_screw_id = 0;
						glm::fvec3 cyl_p[2] = { pos_tool_tip, pos_tool_tip - tool_dir * screw_length };
						glm::fvec3 cyl_rgb = glm::fvec3(1, 0, 1);
						float cyl_r = 0.005f;
						vzm::ObjStates tool_screw;
						vzm::GenerateCylindersObject((float*)cyl_p, &cyl_r, __FP cyl_rgb, 1, tool_screw_id);
						vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, tool_screw_id, tool_screw);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, tool_screw_id, tool_screw);
						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, tool_screw_id, tool_screw);
					}

					glm::fvec3 pos_guide_line = guide_lines[2 * line_guide_idx + 0];
					glm::fvec3 dir_guide_line = guide_lines[2 * line_guide_idx + 1];

					glm::fvec3 closetPoint;
					ComputeClosestPointBetweenLineAndPoint(pos_guide_line, dir_guide_line, pos_tool_tip, closetPoint);

					// show dist line
					static int closest_dist_line_id = 0, closest_dist_text_id = 0;
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
					static int angle_id = 0, angle_text_id = 0, angle_text_id_stg = 0;
					MakeAngle2(pos_tool_tip, tool_dir, closetPoint, 0.05, 0.1, angle_id, ginfo.rs_scene_id, angle_text_id, ginfo.stg_scene_id, angle_text_id_stg);
					vzm::ObjStates angle_state;
					angle_state.color[3] = 0.7;
					vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, angle_id, angle_state);
					vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, angle_text_id_stg, angle_state);
					vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, angle_id, angle_state);
					vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, angle_text_id, angle_state);
					vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, angle_id, angle_state);
					vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, angle_text_id, angle_state);
					// show tool line
					static int tool_id = 0, tool_tip_id = 0;
					glm::fvec3 cyl_rgb = glm::fvec3(1, 1, 0);
					glm::fvec3 cyl_p[2] = { pos_tool_tip, pos_tool_tip + tool_dir * 20.f };
					float cyl_r = 0.005f;

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
					// show MPR
					var_settings::SetSectionalImageAssets(true, __FP pos_tool_tip, __FP (pos_tool_tip + tool_dir * 20.f));
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