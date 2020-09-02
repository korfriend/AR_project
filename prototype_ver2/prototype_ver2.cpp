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

#include "Simulation.h"

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

	var_settings::InitializeVarSettings();
	var_settings::SetCvWindows();
	var_settings::SetPreoperations(rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);

	optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("ss_tool_v1", 0.1f, 1);
	int postpone = 0;
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

	// ssu ///////////////////////////////////////////////////////////////
	bool bSaveGuideFile = false;
	bool guide_toggle = false;

	vector<glm::fvec3> tool_guide_pos_os;
	string modelRootPath("..\\Data");

	var_settings::SetCvWindows_SSU();
	var_settings::SetPreoperations_SSU(modelRootPath);

	// simulation
	Simulation s;
	s.initSSUDeform(modelRootPath.c_str());
	s.initTool(modelRootPath.c_str());

	double dTimeStepMilliSecond = s.getTimeStep() * 1000.0;
	LARGE_INTEGER iT1, iT2;
	LARGE_INTEGER iTFrequency;
	QueryPerformanceFrequency(&iTFrequency);

	std::atomic_bool ssu_deform_alive{ true };
	std::thread deform_processing_thread([&]() {
		while (ssu_deform_alive) {

			if (bAlign) {
				// Simulation
				QueryPerformanceCounter(&iT1);
				s.stepPhysics();

				QueryPerformanceCounter(&iT2);
				// 
				double dSimulationTime = (iT2.QuadPart - iT1.QuadPart) * 1000.0 / iTFrequency.QuadPart;
				double dt = dTimeStepMilliSecond - dSimulationTime;
				iT1 = iT2;

				s.accumulateTime(dSimulationTime);

				if (dt > 0) {
					//Sleep(dt);
				}
			}
		}
	});
	
	//////////////////////////////////////////////////////////////////////
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
		bool load_calib_info = false;
		bool load_stg_calib_info = false;
		bool reset_calib = false;
		bool write_recoded_info = false;
		bool recompile_hlsl = false;
		switch (key_pressed) // http://www.asciitable.com/
		{
		case '[': postpone = max(postpone - 1, 0); cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
		case ']': postpone += 1; cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
		case 'r': recompile_hlsl = true; cout << "Recompile Shader!" << endl; break;
		case 'l': load_calib_info = true; break;
		case 'g': load_stg_calib_info = true; break;
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

		case 'a': guide_toggle = !guide_toggle;	break;


			// RsMouseMode
			//case '1': g_info.touch_mode = RsTouchMode::None; break; 
			//case '2': g_info.touch_mode = RsTouchMode::Pick; break; 
			//case '3': g_info.touch_mode = RsTouchMode::Calib_TC; break; 
			//case '4': g_info.touch_mode = RsTouchMode::PIN_ORIENTATION; break; 
			//case '5': g_info.touch_mode = RsTouchMode::Calib_STG; break; 
		}
		vzm::DebugTestSet("_bool_ReloadHLSLObjFiles", &recompile_hlsl, sizeof(bool), -1, -1);
		vzm::DebugTestSet("_bool_PrintOutRoutineObjs", &show_apis_console, sizeof(bool), -1, -1);
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

			var_settings::SetTargetModelAssets("ss_head", show_csection); // "breastbody"

			// SS tool custom vis.
			{
				GlobalInfo ginfo;
				var_settings::GetVarInfo(&ginfo);

				
				glm::fmat4x4 mat_sstool2ws, mat_sshead2ws, mat_probe2ws;

				bool is_sshead_detected = trk_info.GetLFrmInfo("ss_head", mat_sshead2ws);
				bool is_probe_detected = trk_info.GetLFrmInfo("probe", mat_probe2ws);
				bool is_sstool_detected = trk_info.GetLFrmInfo("ss_tool_v1", mat_sstool2ws);

				if (is_sshead_detected) {
					if (ginfo.is_modelaligned) {
						// deform ¹Ý¿µ //////////////////////////////////
						glm::fvec3 *pos_xyz_list, *nrl_xyz_list;
						unsigned int* idx_prims;
						int num_vtx, num_prims, stride_idx;
						glm::fmat3x3 mat_s;

						// brain
						vzm::GetPModelData(ginfo.brain_ws_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);

						for (int i = 0, ni = s.softBodies[0]->m_surfaceMeshFace.size(); i < ni; i++) {
							const CiSoftBody::Face&	f = s.softBodies[0]->m_surfaceMeshFace[i];
							const btVector3			x[] = { f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x };

							glm::fvec3 v0 = glm::fvec3(x[0].getX(), x[0].getY(), x[0].getZ());
							glm::fvec3 v1 = glm::fvec3(x[1].getX(), x[1].getY(), x[1].getZ());
							glm::fvec3 v2 = glm::fvec3(x[2].getX(), x[2].getY(), x[2].getZ());

							int j = i * 3;
							pos_xyz_list[j] = v0;
							pos_xyz_list[j + 1] = v1;
							pos_xyz_list[j + 2] = v2;
						}
						vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, ginfo.brain_ws_obj_id);
						delete[] pos_xyz_list;
						delete[] nrl_xyz_list;
						delete[] idx_prims;

						// ventricle
						vzm::GetPModelData(ginfo.ventricle_ws_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);
						for (int c = 0; c < s.softBodies[0]->m_childCnt; c++) {
							for (int i = 0, ni = s.softBodies[0]->m_child[c].m_surfaceMeshFace.size(); i < ni; i++) {
								const CiSoftBody::Face&	f = s.softBodies[0]->m_child[c].m_surfaceMeshFace[i];
								const btVector3			x[] = { f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x };

								glm::fvec3 v0 = glm::fvec3(x[0].getX(), x[0].getY(), x[0].getZ());
								glm::fvec3 v1 = glm::fvec3(x[1].getX(), x[1].getY(), x[1].getZ());
								glm::fvec3 v2 = glm::fvec3(x[2].getX(), x[2].getY(), x[2].getZ());

								int j = i * 3;
								pos_xyz_list[j] = v0;
								pos_xyz_list[j + 1] = v1;
								pos_xyz_list[j + 2] = v2;
							}
						}
						vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, ginfo.ventricle_ws_obj_id);
						delete[] pos_xyz_list;
						delete[] nrl_xyz_list;
						delete[] idx_prims;

						// replace scene object
						vzm::ObjStates model_ws_states, brain_ws_states, ventricle_ws_states;
						vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_states);

						model_ws_states.color[3] = 0.1;

						brain_ws_states = model_ws_states;
						brain_ws_states.is_wireframe = true;
						brain_ws_states.wire_color[0] = 0.5; brain_ws_states.wire_color[1] = 0.5; brain_ws_states.wire_color[2] = 0.5; brain_ws_states.wire_color[3] = 0.2;
						brain_ws_states.color[0] = 0.5; brain_ws_states.color[1] = 0.5; brain_ws_states.color[2] = 0.5; brain_ws_states.color[3] = 0.3;

						ventricle_ws_states = model_ws_states;
						ventricle_ws_states.color[0] = 1.0; ventricle_ws_states.color[1] = 0; ventricle_ws_states.color[2] = 0; ventricle_ws_states.color[3] = 1.0;

						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ginfo.model_ws_obj_id, model_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.zoom_scene_id, ginfo.model_ws_obj_id, model_ws_states);

						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ginfo.brain_ws_obj_id, brain_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ginfo.brain_ws_obj_id, brain_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.zoom_scene_id, ginfo.brain_ws_obj_id, brain_ws_states);

						vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ginfo.ventricle_ws_obj_id, ventricle_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ginfo.ventricle_ws_obj_id, ventricle_ws_states);
						vzm::ReplaceOrAddSceneObject(ginfo.zoom_scene_id, ginfo.ventricle_ws_obj_id, ventricle_ws_states);
					}
				}

				static int probe_line_id = 0, probe_end_id = 0;
				if (is_probe_detected) {

					// probe visualization //
					glm::fvec3 probe_end = tr_pt(mat_probe2ws, glm::fvec3(0));
					glm::fvec3 probe_dir = glm::normalize(tr_vec(mat_probe2ws, glm::fvec3(0, 0, -1)));

					// probe line
					glm::fvec3 cyl_p01[2] = { probe_end, probe_end - probe_dir * 0.2f };
					float cyl_r = 0.0015f;
					glm::fvec3 cyl_rgb = glm::fvec3(0, 1, 1);
					vzm::GenerateCylindersObject((float*)cyl_p01, &cyl_r, __FP cyl_rgb, 1, probe_line_id);

					// replace scene object
					vzm::ObjStates model_ws_states;
					vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_states);
					vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, probe_line_id, model_ws_states);
					vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, probe_line_id, model_ws_states);
				}


				static int ssu_tool_line_id = 0, ssu_tool_end_id = 0;
				static int ssu_tool_end_zs_id = 0;
				if (is_sstool_detected && ss_tool_info.pos_centers_tfrm.size() > 0)
				{
					// ssu tool visualization //
					glm::fvec3 sstool_p1_ws = tr_pt(mat_sstool2ws, ss_tool_info.pos_centers_tfrm[0]);
					glm::fvec3 sstool_p2_ws = tr_pt(mat_sstool2ws, ss_tool_info.pos_centers_tfrm[1]);

					glm::fvec3 sstool_dir = glm::normalize(sstool_p2_ws - sstool_p1_ws);
					sstool_p2_ws = sstool_p1_ws + sstool_dir * 0.2f;

					// tool line (ws, rs)
					glm::fvec3 cyl_p01[2] = { sstool_p1_ws, sstool_p1_ws + sstool_dir * 0.2f };
					float cyl_r = 0.0015f;
					glm::fvec3 cyl_rgb = glm::fvec3(0, 1, 1);
					vzm::GenerateCylindersObject((float*)cyl_p01, &cyl_r, __FP cyl_rgb, 1, ssu_tool_line_id);

					// sphere (ws, rs)
					vzm::GenerateSpheresObject(__FP glm::fvec4(sstool_p1_ws, 0.0045f), __FP glm::fvec3(1, 0, 0), 1, ssu_tool_end_id);

					// replace scene object
					vzm::ObjStates model_ws_states;
					vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_states);
					vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ssu_tool_line_id, model_ws_states);
					vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ssu_tool_line_id, model_ws_states);

					vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ssu_tool_end_id, model_ws_states);
					vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ssu_tool_end_id, model_ws_states);


					// zoom scene //
					if (ginfo.is_modelaligned) {
						// sphere (zs)
						vzm::GenerateSpheresObject(__FP glm::fvec4(sstool_p1_ws, 0.0015f), __FP glm::fvec3(0, 1, 1), 1, ssu_tool_end_zs_id);
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