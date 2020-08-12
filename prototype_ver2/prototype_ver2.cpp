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

#include "../aruco_marker/aruco_armarker.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "VisMtvApi.h"
#include "../optitrk/optitrack.h"

#define __NUMMARKERS 15

using namespace std;
using namespace cv;

#include "../kar_helpers.hpp"
#include "MouseEvents.hpp"

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

GlobalInfo g_info;

struct SS_Tool_Guide_Pts
{
	int ss_tool_guide_points_id;
	vector<glm::fvec3> pos_centers_tfrm;
	SS_Tool_Guide_Pts() { ss_tool_guide_points_id = 0; }
};
SS_Tool_Guide_Pts ss_tool_info;

ArMarkerTracker ar_marker;

int main()
{
	// set global information
	g_info.ws_scene_id = 1;
	g_info.rs_scene_id = 2;
	g_info.model_scene_id = 3;
	g_info.csection_scene_id = 4;

	g_info.window_name_rs_view = "RealSense VIEW";
	g_info.window_name_ws_view = "World VIEW";
	g_info.window_name_ms_view = "Model VIEW";

	// load txt file
	g_info.optrack_calib = "D:\\Document\\OptiTrack\\my_test_200812_1.cal";
	g_info.optrack_env = "D:\\Document\\OptiTrack\\my_test_200812_1.motive";
	g_info.cb_positions = "E:\\project_srcs\\kar\\prototype_ver1\\cb_points.txt";
	g_info.sst_positions = "E:\\project_srcs\\kar\\prototype_ver1\\ss_pin_pts.txt";
	//g_info.model_path = "D:\\Data\\K-AR_Data\\demo.obj";
	g_info.model_path = "..\\Data\\skin.obj";
//	g_info.model_path = "D:\\Data\\K-AR_Data\\brain\\1\\skin_c_output.obj";
	//g_info.model_path = "D:\\Data\\K-AR_Data\\chest_x3d\\chest_x3d.x3d";

#if defined(_DEBUG) | defined(DEBUG)
	//_CrtSetDbgFlag(_CRTDBG_ALLOC_MEM_DF | _CRTDBG_LEAK_CHECK_DF);
#endif	

	set<int> mk_ids;
	for (int i = 1; i <= __NUMMARKERS; i++)
	{
		mk_ids.insert(i);
		ar_marker.register_marker(i, 5.15);
		//ar_marker.aruco_marker_file_out(i, "armk" + to_string(i) + ".bmp");
	}
	glm::fvec3 sphere_clrs[__NUMMARKERS] = {
		glm::fvec3(1, 0.3, 0.3), glm::fvec3(0.3, 1, 0.3), glm::fvec3(0.3, 0.3, 1),
		glm::fvec3(1, 1, 0.3), glm::fvec3(0.3, 1, 1), glm::fvec3(1, 0.3, 1) };

	vzm::InitEngineLib();

	
	vzm::LoadModelFile(g_info.model_path, g_info.model_obj_id);
	g_info.is_meshmodel = true;
	vzm::ValidatePickTarget(g_info.model_obj_id);
	int model_obj_ws_id = 0;
	vzm::GenerateCopiedObject(g_info.model_obj_id, model_obj_ws_id);


	/////////////////////////////////////////////////////////////////////////
	Simulation s;
	string modelRootPath("..\\Data");
	s.initSSUDeform(modelRootPath.c_str());
	s.initTool(modelRootPath.c_str());


	string brainPath = modelRootPath + "\\brain.obj";
	vzm::LoadModelFile(brainPath, g_info.brain_obj_id);

	string ventriclePath = modelRootPath + "\\ventricle.obj";
	vzm::LoadModelFile(ventriclePath, g_info.ventricle_obj_id);

	/////////////////////////////////////////////////////////////////////////


	vzm::CameraParameters cam_params;
	if (!optitrk::InitOptiTrackLib())
	{
		printf("Unable to license Motive API\n");
		return 1;
	}
	optitrk::LoadProfileAndCalibInfo(g_info.optrack_env, g_info.optrack_calib);
	cout << "cam0 frame rate setting ==> " << optitrk::SetCameraFrameRate(0, 120) << endl;
	cout << "cam1 frame rate setting ==> " << optitrk::SetCameraFrameRate(1, 120) << endl;

	__cv3__ cam_params.pos = glm::fvec3(1.0, 2.0, 1.5f);
	glm::fvec3 t_up = glm::fvec3(0, 1.f, 0);
	__cv3__ cam_params.view = glm::normalize(glm::fvec3(0, 1, 0) - __cv3__ cam_params.pos);
	glm::fvec3 t_right = glm::cross(__cv3__ cam_params.view, t_up);
	__cv3__ cam_params.up = glm::normalize(glm::cross(t_right, __cv3__ cam_params.view));

	cam_params.fov_y = 3.141592654f / 4.f;
	cam_params.aspect_ratio = 640.f / 480.f;
	cam_params.projection_mode = 2;
	cam_params.w = 1024;
	cam_params.h = 640;
	cam_params.np = 0.1f;
	cam_params.fp = 20.0f;

	int ov_cam_id = 1; // arbitrary integer
	vzm::SetCameraParameters(g_info.ws_scene_id, cam_params, ov_cam_id);

	int model_cam_id = 1; // arbitrary integer

	vzm::CameraParameters cam_params_model = cam_params;
	cam_params_model.np = 0.01f;
	cam_params_model.fp = 10.0f;
	__cv3__ cam_params_model.pos = glm::fvec3(0.3f, 0, 0);
	__cv3__ cam_params_model.up = glm::fvec3(0, 1.f, 0);
	__cv3__ cam_params_model.view = glm::fvec3(-1.f, 0, 0.f);
	vzm::SetCameraParameters(g_info.model_scene_id, cam_params_model, model_cam_id);

	vzm::SceneEnvParameters scn_env_params;
	scn_env_params.is_on_camera = false;
	scn_env_params.is_pointlight = false;
	scn_env_params.effect_ssao.is_on_ssao = false;
	scn_env_params.effect_ssao.kernel_r = 0.01f;
	scn_env_params.effect_ssao.num_dirs = 16;
	__cv3__ scn_env_params.pos_light = __cv3__ cam_params.pos;
	__cv3__ scn_env_params.dir_light = __cv3__ cam_params.view;
	vzm::SetSceneEnvParameters(g_info.ws_scene_id, scn_env_params);
	scn_env_params.is_on_camera = true;
	vzm::SetSceneEnvParameters(g_info.rs_scene_id, scn_env_params);
	vzm::SceneEnvParameters ms_scn_env_params = scn_env_params;
	ms_scn_env_params.is_on_camera = true;
	vzm::SetSceneEnvParameters(g_info.model_scene_id, ms_scn_env_params);
	vzm::SceneEnvParameters csection_scn_env_params = scn_env_params;
	vzm::SetSceneEnvParameters(g_info.csection_scene_id, csection_scn_env_params);

	vzm::ObjStates obj_state;
	obj_state.emission = 0.4f;
	obj_state.diffusion = 0.6f;
	obj_state.specular = 0.2f;
	obj_state.sp_pow = 30.f;
	__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
	__cm4__ obj_state.os2ws = glm::fmat4x4();

	double vz = 0.001;
	vzm::DebugTestSet("_double_VZThickness", &vz, sizeof(double), -1, -1);
	double cvz = 0.0005;
	vzm::DebugTestSet("_double_CopVZThickness", &cvz, sizeof(double), -1, -1);
	//srand(0);
	//for (int i = 0; i < 100; i++)
	//{
	//	glm::fvec4 xyzr = glm::fvec4((rand() % 1000 - 500.f) * 0.001f, (rand() % 1000 - 500.f) * 0.001f, (rand() % 2000) * 0.001f + 1.0f, (rand() % 1000) * 0.00005f);
	//	glm::fvec3 rgb = glm::fvec3((rand() % 255) / 255.f, (rand() % 255) / 255.f, (rand() % 255) / 255.f);
	//	int sp_obj_id = 0;
	//	vzm::GenerateSpheresObject(__FP xyzr, __FP rgb, 1, sp_obj_id);
	//	obj_state.use_vertex_color = true;
	//	obj_state.color[3] = 0.7f;
	//	vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, sp_obj_id, obj_state);
	//}

	bool use_new_version = true;
	vzm::DebugTestSet("_bool_TestOit", &use_new_version, sizeof(bool), -1, -1); cout << "Use Prev Version : OFF" << endl;

	std::vector< glm::fvec3> tx_loc_info;
	tx_loc_info.push_back(glm::fvec3(0, 100, 0));
	tx_loc_info.push_back(glm::fvec3(0, 0, 1));
	tx_loc_info.push_back(glm::fvec3(0, 1, 0));
	int text_id = 0;
	vzm::GenerateTextObject((float*)&tx_loc_info[0], "TEST ABC", 30.f, true, false, text_id);
	//vzm::ReplaceOrAddSceneObject(vr_scene_id, text_id, obj_state);

	//Create a window
	cv::namedWindow(g_info.window_name_rs_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);
	cv::namedWindow(g_info.window_name_ws_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);
	cv::namedWindow(g_info.window_name_ms_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);

	vzm::ObjStates model_state = obj_state;
	model_state.color[3] = 0.8;
	glm::fmat4x4 mat_s = glm::scale(glm::fvec3(0.001));
	__cm4__ model_state.os2ws = (__cm4__ model_state.os2ws) * mat_s;
	//if (g_info.is_meshmodel)
	{
		int vr_tmap_id, mpr_tmap_id;
		std::vector<glm::fvec2> alpha_ctrs;
		alpha_ctrs.push_back(glm::fvec2(0, 17760));
		alpha_ctrs.push_back(glm::fvec2(1, 21700));
		alpha_ctrs.push_back(glm::fvec2(1, 65536));
		alpha_ctrs.push_back(glm::fvec2(0, 65537));
		std::vector<glm::fvec4> rgb_ctrs;
		rgb_ctrs.push_back(glm::fvec4(1, 1, 1, 0));
		rgb_ctrs.push_back(glm::fvec4(0.31, 0.78, 1, 17760));
		rgb_ctrs.push_back(glm::fvec4(1, 0.51, 0.49, 18900));
		rgb_ctrs.push_back(glm::fvec4(1, 1, 1, 21000));
		rgb_ctrs.push_back(glm::fvec4(1, 1, 1, 65536));
		vzm::GenerateMappingTable(65537, alpha_ctrs.size(), (float*)&alpha_ctrs[0], rgb_ctrs.size(), (float*)&rgb_ctrs[0], vr_tmap_id);

		alpha_ctrs[0] = glm::fvec2(0, 100);
		alpha_ctrs[1] = glm::fvec2(1, 30000);
		rgb_ctrs[1] = glm::fvec4(1);
		rgb_ctrs[2] = glm::fvec4(1);
		vzm::GenerateMappingTable(65537, alpha_ctrs.size(), (float*)&alpha_ctrs[0], rgb_ctrs.size(), (float*)&rgb_ctrs[0], mpr_tmap_id);
		model_state.associated_obj_ids["VR_OTF"] = vr_tmap_id;
		model_state.associated_obj_ids["MPR_WINDOWING"] = mpr_tmap_id;
	}
	vzm::ReplaceOrAddSceneObject(g_info.model_scene_id, g_info.model_obj_id, model_state);
	Show_Window(g_info.window_name_ms_view, g_info.model_scene_id, model_cam_id);

	// Colorizer is used to visualize depth data
	rs2::colorizer color_map;
	// Use black to white color map
	color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);
	// Decimation filter reduces the amount of data (while preserving best samples)
	rs2::decimation_filter dec;
	// If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
	// but you can also increase the following parameter to decimate depth more (reducing quality)
	dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
	// Define transformations from and to Disparity domain
	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth(false);
	// Define spatial filter (edge-preserving)
	rs2::spatial_filter spat;
	// Enable hole-filling
	// Hole filling is an agressive heuristic and it gets the depth wrong many times
	// However, this demo is not built to handle holes
	// (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
	spat.set_option(RS2_OPTION_HOLES_FILL, 5); // 5 = fill all the zero pixels
	// Define temporal filter
	rs2::temporal_filter temp;
	// Spatially align all streams to depth viewport
	// We do this because:
	//   a. Usually depth has wider FOV, and we only really need depth for this demo
	//   b. We don't want to introduce new holes
	rs2::align align_to(RS2_STREAM_DEPTH);
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::pipeline pipe;

	rs2::config cfg;
	cfg.enable_stream(RS2_STREAM_DEPTH); // Enable default depth
	// For the color stream, set format to RGBA
	// To allow blending of the color frame on top of the depth frame
	cfg.enable_stream(RS2_STREAM_COLOR, RS2_FORMAT_RGB8);

	auto profile = pipe.start(cfg);

	auto sensor = profile.get_device().first<rs2::depth_sensor>();
	// Set the device to High Accuracy preset of the D400 stereoscopic cameras
	if (sensor && sensor.is<rs2::depth_stereo_sensor>())
	{
		sensor.set_option(RS2_OPTION_VISUAL_PRESET, RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY);
		sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1);
	}

	// to get rgb sensor profile
	auto stream_rgb = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	auto stream_depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	rs2_intrinsics rgb_intrinsics;
	if (stream_rgb)
	{
		rgb_intrinsics = stream_rgb.get_intrinsics();
		cout << std::endl << "RGB Intrinsic:," << std::endl;
		cout << "Fx," << rgb_intrinsics.fx << std::endl;
		cout << "Fy," << rgb_intrinsics.fy << std::endl;
		cout << "PPx," << rgb_intrinsics.ppx << std::endl;
		cout << "PPy," << rgb_intrinsics.ppy << std::endl;
		cout << "Distorsion," << rs2_distortion_to_string(rgb_intrinsics.model) << std::endl;
	}
	rs2_extrinsics rgb_extrinsics = stream_depth.get_extrinsics_to(stream_rgb);

	rs2_intrinsics depth_intrinsics;
	if (stream_depth)
	{
		depth_intrinsics = stream_depth.get_intrinsics();
		cout << std::endl << "Depth Intrinsic:," << std::endl;
		cout << "Fx," << depth_intrinsics.fx << std::endl;
		cout << "Fy," << depth_intrinsics.fy << std::endl;
		cout << "PPx," << depth_intrinsics.ppx << std::endl;
		cout << "PPy," << depth_intrinsics.ppy << std::endl;
		cout << "Distorsion," << rs2_distortion_to_string(depth_intrinsics.model) << std::endl;
	}

	vzm::CameraParameters rs_cam_params;
	__cv3__ rs_cam_params.pos = glm::fvec3(0);
	__cv3__ rs_cam_params.up = glm::fvec3(0, 1, 0);
	__cv3__ rs_cam_params.view = glm::fvec3(0, 0, 1);

	rs_cam_params.fx = rgb_intrinsics.fx;
	rs_cam_params.fy = rgb_intrinsics.fy;
	rs_cam_params.cx = rgb_intrinsics.ppx;
	rs_cam_params.cy = rgb_intrinsics.ppy;
	rs_cam_params.sc = 0;
	rs_cam_params.w = rgb_intrinsics.width;
	rs_cam_params.h = rgb_intrinsics.height;
	rs_cam_params.np = 0.1f;
	rs_cam_params.fp = 20.0f;
	rs_cam_params.projection_mode = 3;

	int rs_cam_id = 1; // arbitrary integer
	vzm::SetCameraParameters(g_info.rs_scene_id, rs_cam_params, rs_cam_id);


	glm::fmat4x4 mat_ircs2irss, mat_irss2ircs;
	if (stream_depth)
	{
		glm::fmat3x3 mat_int;
		mat_int[0][0] = depth_intrinsics.fx;
		mat_int[1][0] = 0;
		mat_int[2][0] = depth_intrinsics.ppx;
		mat_int[1][1] = depth_intrinsics.fy;
		mat_int[2][1] = depth_intrinsics.ppy;

		glm::fmat4x4 mat_cs2ps, mat_ps2ss;
		ComputeDXProjectionMatrix((float*)&mat_cs2ps, (float*)&mat_ps2ss, (float*)&mat_int,
			0, 0, (float)rgb_intrinsics.width, (float)rgb_intrinsics.height, 0.01f, 10.f);
		mat_ircs2irss = mat_ps2ss * mat_cs2ps;
		mat_irss2ircs = glm::inverse(mat_ircs2irss);
	}

	//auto stream = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	//auto stream = profile.get_stream(RS2_STREAM_COLOR | RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();

	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	// After initial post-processing, frames will flow into this queue:
	rs2::frame_queue original_data;
	rs2::frame_queue filtered_data;
	//rs2::frame_queue postprocessed_depthframes(1);

	// Alive boolean will signal the worker threads to finish-up
	std::atomic_bool rs_alive{ true };
	std::thread video_processing_thread([&]() {
		while (rs_alive)
		{
			// Fetch frames from the pipeline and send them for processing
			//rs2::frameset data;
			//if (pipe.poll_for_frames(&data))
			{
				rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

				rs2::frame data_depth = data.get_depth_frame();
				if (!data_depth) // Should not happen but if the pipeline is configured differently
					return;       //  it might not provide depth and we don't want to crash
				if (data_depth != NULL)
				{
					// First make the frames spatially aligned
					data_depth = data_depth.apply_filter(align_to);

					// Decimation will reduce the resultion of the depth image,
					// closing small holes and speeding-up the algorithm
					data_depth = data_depth.apply_filter(dec);

					// To make sure far-away objects are filtered proportionally
					// we try to switch to disparity domain
					data_depth = data_depth.apply_filter(depth2disparity);

					// Apply spatial filtering
					data_depth = data_depth.apply_filter(spat);

					// Apply temporal filtering
					data_depth = data_depth.apply_filter(temp);

					// If we are in disparity domain, switch back to depth
					data_depth = data_depth.apply_filter(disparity2depth);

					// Apply color map for visualization of depth
					// data = data.apply_filter(color_map);
					// data.arbitrary_depth = data_depth;
					// postprocessed_depthframes.enqueue(data_depth);
					filtered_data.enqueue(data_depth);
					original_data.enqueue(data);
				}

				// Send resulting frames for visualization in the main thread
				//original_data.enqueue(data);
			}
		}
	});

	EventGlobalInfo rg_info_world(g_info, g_info.ws_scene_id, ov_cam_id);
	cv::setMouseCallback(g_info.window_name_ws_view, CallBackFunc_WorldMouse, &rg_info_world);
	EventGlobalInfo rg_info_model(g_info, g_info.model_scene_id, model_cam_id);
	cv::setMouseCallback(g_info.window_name_ms_view, CallBackFunc_ModelMouse, &rg_info_model);

	EventGlobalInfo rg_info_rs(g_info, 0, 0);
	cv::setMouseCallback(g_info.window_name_rs_view, CallBackFunc_RsMouse, &rg_info_rs);

	optitrk::UpdateFrame();
	glm::fmat4x4 mat_cam0_to_ws, mat_cam1_to_ws;
	optitrk::GetCameraLocation(0, (float*)&mat_cam0_to_ws);
	optitrk::GetCameraLocation(1, (float*)&mat_cam1_to_ws);

	Register_CamModel(g_info.ws_scene_id, mat_cam0_to_ws, "IR CAM 0", 0);
	Register_CamModel(g_info.ws_scene_id, mat_cam1_to_ws, "IR CAM 1", 1);

	optitrk::SetRigidBodyPropertyByName("rs_cam", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("probe", 0.1f, 1);
	optitrk::SetRigidBodyPropertyByName("ss_tool_v1", 0.1f, 1);

	static int postpone = 4;
	concurrent_queue<track_info> track_que(10);
	std::atomic_bool tracker_alive{ true };
	std::thread tracker_processing_thread([&]() {
		while (tracker_alive)
		{
			Sleep(postpone);
			optitrk::UpdateFrame();

			track_info cur_trk_info;
			cur_trk_info.is_detected_rscam = optitrk::GetRigidBodyLocationByName("rs_cam", (float*)&cur_trk_info.mat_rbcam2ws);
			cur_trk_info.is_detected_probe = optitrk::GetRigidBodyLocationByName("probe", (float*)&cur_trk_info.mat_probe2ws);
			cur_trk_info.is_detected_sstool = optitrk::GetRigidBodyLocationByName("ss_tool_v1", (float*)&cur_trk_info.mat_tfrm2ws);
			cur_trk_info.is_detected_sshead = optitrk::GetRigidBodyLocationByName("ss_head", (float*)&cur_trk_info.mat_headfrm2ws);
			cur_trk_info.is_detected_brbody = optitrk::GetRigidBodyLocationByName("breast_body", (float*)&cur_trk_info.mat_bodyfrm2ws);

			//cout << cur_trk_info.is_detected_rscam << ", " << cur_trk_info.is_detected_probe << endl;

			optitrk::GetMarkersLocation(&cur_trk_info.mk_xyz_list, &cur_trk_info.mk_residue_list);
			cur_trk_info.is_updated = true;
			track_que.push(cur_trk_info);
		}
	});

	//////////////////////////////////////////////////////////////////
	double dTimeStepMilliSecond = s.getTimeStep() * 1000.0;
	LARGE_INTEGER iT1, iT2;
	LARGE_INTEGER iTFrequency;
	QueryPerformanceFrequency(&iTFrequency);

	std::atomic_bool ssu_deform_alive{ true };
	std::thread deform_processing_thread([&]() {
		while (ssu_deform_alive && g_info.align_matching_model) {
			// Simulation
			QueryPerformanceCounter(&iT1);
			s.stepPhysics();

			glm::fvec3 *pos_xyz_list, *nrl_xyz_list;
			unsigned int* idx_prims;
			int num_vtx, num_prims, stride_idx;
			glm::fvec3 *rgb_list;
			glm::fmat3x3 mat_s;

			// brain //
			vzm::GetPModelData(g_info.brain_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);
			rgb_list = new glm::fvec3[num_vtx];
			mat_s = glm::scale(glm::fvec3(1.f, 0.5f, 0.5f));
			for (int i = 0, ni = s.softBodies[0]->m_surfaceMeshFace.size(); i < ni; i++) {
				const CiSoftBody::Face&	f = s.softBodies[0]->m_surfaceMeshFace[i];
				const btVector3			x[] = { f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x };

				/*
				glm::fvec3 v0 = glm::fvec3(x[0].getX() / 1000.0000, x[0].getY() / 1000.0000, x[0].getZ() / 1000.0000);
				glm::fvec3 v1 = glm::fvec3(x[1].getX() / 1000.0000, x[1].getY() / 1000.0000, x[1].getZ() / 1000.0000);
				glm::fvec3 v2 = glm::fvec3(x[2].getX() / 1000.0000, x[2].getY() / 1000.0000, x[2].getZ() / 1000.0000);
				*/
				glm::fvec3 v0 = glm::fvec3(x[0].getX(), x[0].getY(), x[0].getZ());
				glm::fvec3 v1 = glm::fvec3(x[1].getX(), x[1].getY(), x[1].getZ());
				glm::fvec3 v2 = glm::fvec3(x[2].getX(), x[2].getY(), x[2].getZ());

				int j = i * 3;
				pos_xyz_list[j] = v0;
				pos_xyz_list[j + 1] = v1;
				pos_xyz_list[j + 2] = v2;

				//rgb_list[j] = glm::fvec4(0.114f, 0.8f, 1.f, 0.45f);
				//rgb_list[j + 1] = glm::fvec4(0.114f, 0.8f, 1.f, 0.45f);
				//rgb_list[j + 2] = glm::fvec4(0.114f, 0.8f, 1.f, 0.45f);
			}
			vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, g_info.brain_obj_id);
			delete[] pos_xyz_list;
			delete[] nrl_xyz_list;
			delete[] idx_prims;
			delete[] rgb_list;


			// ventricle //
			vzm::GetPModelData(g_info.ventricle_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);
			rgb_list = new glm::fvec3[num_vtx];
			mat_s = glm::scale(glm::fvec3(1.f, 0.5f, 0.5f));
			for (int c = 0; c < s.softBodies[0]->m_childCnt; c++) {
				for (int i = 0, ni = s.softBodies[0]->m_child[c].m_surfaceMeshFace.size(); i < ni; i++) {
					const CiSoftBody::Face&	f = s.softBodies[0]->m_child[c].m_surfaceMeshFace[i];
					const btVector3			x[] = { f.m_n[0]->m_x,f.m_n[1]->m_x,f.m_n[2]->m_x };

					/*
					glm::fvec3 v0 = glm::fvec3(x[0].getX() / 1000.0000, x[0].getY() / 1000.0000, x[0].getZ() / 1000.0000);
					glm::fvec3 v1 = glm::fvec3(x[1].getX() / 1000.0000, x[1].getY() / 1000.0000, x[1].getZ() / 1000.0000);
					glm::fvec3 v2 = glm::fvec3(x[2].getX() / 1000.0000, x[2].getY() / 1000.0000, x[2].getZ() / 1000.0000);
					*/
					glm::fvec3 v0 = glm::fvec3(x[0].getX(), x[0].getY(), x[0].getZ());
					glm::fvec3 v1 = glm::fvec3(x[1].getX(), x[1].getY(), x[1].getZ());
					glm::fvec3 v2 = glm::fvec3(x[2].getX(), x[2].getY(), x[2].getZ());

					int j = i * 3;
					pos_xyz_list[j] = v0;
					pos_xyz_list[j + 1] = v1;
					pos_xyz_list[j + 2] = v2;

					//rgb_list[j] = glm::fvec4(0.164f, 0.164f, 0.92f, 0.66f);
					//rgb_list[j + 1] = glm::fvec4(0.164f, 0.164f, 0.92f, 0.66f);
					//rgb_list[j + 2] = glm::fvec4(0.164f, 0.164f, 0.92f, 0.66f);
				}
			}

			vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, g_info.ventricle_obj_id);



			QueryPerformanceCounter(&iT2);
			// 
			double dSimulationTime = (iT2.QuadPart - iT1.QuadPart) * 1000.0 / iTFrequency.QuadPart;
			double dt = dTimeStepMilliSecond - dSimulationTime;

			if (dt > 0) {
				Sleep(dt);
			}
		}
	});
	//////////////////////////////////////////////////////////////////


	// make 3d ui widgets
	int coord_grid_obj_id = 0, axis_lines_obj_id = 0, axis_texX_obj_id = 0, axis_texZ_obj_id = 0;
	World_GridAxis_Gen(coord_grid_obj_id, axis_lines_obj_id, axis_texX_obj_id, axis_texZ_obj_id);
	vzm::ObjStates grid_obj_state;
	grid_obj_state.color[3] = 0.7f;
	grid_obj_state.line_thickness = 0;
	vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, coord_grid_obj_id, grid_obj_state);
	bool foremost_surf_rendering = true;
	vzm::DebugTestSet("_bool_OnlyForemostSurfaces", &foremost_surf_rendering, sizeof(bool), g_info.ws_scene_id, ov_cam_id, coord_grid_obj_id);
	grid_obj_state.color[3] = 0.9f;
	vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, axis_lines_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(1, 0.3, 0.3, 0.6);
	vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, axis_texX_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(0.3, 0.3, 1, 0.6);
	vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, axis_texZ_obj_id, grid_obj_state);

	// params for the main thread
	int key_pressed = -1;
	float pnp_err = -1.f;
	bool recompile_hlsl = false;
	bool show_apis_console = false;
	bool show_csection = false;
	bool show_mks = true;
	bool do_initialize_trk_points = false;
	bool show_calib_frames = true;
	int rs_lf_axis = 0, probe_lf_axis = 0, sstool_lf_axis = 0; // lf means local frame
	bool show_pc = false;
	bool calib_toggle = false;
	int num_calib = 0;
	int calib_samples = 0;
	glm::fmat4x4 mat_rscs2clf;

	while (key_pressed != 'q' && key_pressed != 27)
	{
		key_pressed = cv::waitKey(1);
		bool load_calib_points = false;
		switch (key_pressed) // http://www.asciitable.com/
		{
		case 91: postpone = max(postpone - 1, 0);  break; // [ 
		case 93: postpone += 1; break; // ] 
		case 97: use_new_version = !use_new_version;  cout << "Use Prev Version : " << use_new_version << endl; break; // a 
		case 114: recompile_hlsl = true; cout << "Recompile Shader!" << endl; break; // r
		case 108: load_calib_points = true; break; // l
		case 118: show_calib_frames = !show_calib_frames; break; // v
		case 112: show_pc = !show_pc; break; // p
		case 101: show_apis_console = !show_apis_console; break; // e
		case 109: show_mks = !show_mks; break; // m
		case 99: calib_toggle = !calib_toggle; break; // c
		case 115: show_csection = !show_csection; break; // s
			// RsMouseMode
		case 49: g_info.rs_ms_mode = RsMouseMode::NONE; break; // 1
		case 50: g_info.rs_ms_mode = RsMouseMode::ADD_CALIB_POINTS; break; // 2
		case 51: g_info.rs_ms_mode = RsMouseMode::GATHERING_POINTS; break; // 3
		case 52: g_info.rs_ms_mode = RsMouseMode::PIN_ORIENTATION; break; // 4
		}

		vzm::DisplayConsoleMessages(show_apis_console);

		if (g_info.skip_main_thread) continue;

		// Fetch the latest available post-processed frameset
		//static rs2::frameset frameset0, frameset1;
		rs2::frameset current_frameset;
		original_data.poll_for_frame(&current_frameset);
		rs2::frame current_depth_frame;
		filtered_data.poll_for_frame(&current_depth_frame);

		track_info trk_info;
		track_que.wait_and_pop(trk_info);
		if (trk_info.is_updated && current_frameset && current_depth_frame)
		{
			glm::fmat4x4 mat_ws2clf, mat_clf2ws;
			g_info.otrk_data.trk_info = trk_info;
			mat_clf2ws = trk_info.mat_rbcam2ws;
			mat_ws2clf = glm::inverse(mat_clf2ws);

			rs2::depth_frame depth_frame = current_depth_frame;// .get_depth_frame();
			auto color = current_frameset.get_color_frame();
			//auto colorized_depth = current_frameset.first(RS2_STREAM_DEPTH, RS2_FORMAT_RGB8);

			const int w = color.as<rs2::video_frame>().get_width();
			const int h = color.as<rs2::video_frame>().get_height();

			Mat image(Size(w, h), CV_8UC3, (void*)color.get_data(), Mat::AUTO_STEP);
			Mat imagebgr;

			cvtColor(image, imagebgr, COLOR_BGR2RGB);

			if (load_calib_points)
			{
				g_info.otrk_data.calib_3d_pts.clear();
				std::ifstream infile(g_info.cb_positions);
				string line;
				while (getline(infile, line))
				{
					std::istringstream iss(line);
					float a, b, c;
					if (!(iss >> a >> b >> c)) { break; } // error
					g_info.otrk_data.calib_3d_pts.push_back(Point3f(a, b, c));
					// process pair (a,b)
				}
				infile.close();

				//if (trk_info.is_detected_sstool)
				//{
				//	ss_tool_info.pos_centers_tfrm.clear();
				//
				//	infile = std::ifstream(sst_positions);
				//	line = "";
				//	while (getline(infile, line))
				//	{
				//		std::istringstream iss(line);
				//		float a, b, c;
				//		if (!(iss >> a >> b >> c)) { break; } // error
				//		ss_tool_info.pos_centers_tfrm.push_back(glm::fvec3(a, b, c));
				//		// process pair (a,b)
				//	}
				//	infile.close();
				//}
			}

			auto marker_color = [](int idx, int w)
			{
				return glm::fvec3((idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1), 1);
			};

			if (g_info.rs_ms_mode == RsMouseMode::ADD_CALIB_POINTS)
			{
				vector<glm::fvec4> sphers_xyzr;
				vector<glm::fvec3> sphers_rgb;
				for (int i = 0; i < g_info.otrk_data.calib_3d_pts.size(); i++)
				{
					Point3f pt = g_info.otrk_data.calib_3d_pts[i];
					sphers_xyzr.push_back(glm::fvec4(pt.x, pt.y, pt.z, 0.005));
					sphers_rgb.push_back(marker_color(i, (int)g_info.otrk_data.calib_3d_pts.size() / 2));
				}
				if (sphers_xyzr.size() > 0)
				{
					vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0],
						g_info.otrk_data.calib_3d_pts.size(), g_info.otrk_data.cb_spheres_id);
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.cb_spheres_id, obj_state);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.cb_spheres_id, obj_state);
				}
				else
				{
					vzm::ObjStates cstate = obj_state;
					cstate.is_visible = false;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
				}
			}

			static vector<int> calib_trial_cam_frame_ids;
			if (do_initialize_trk_points)
			{
				for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
					vzm::DeleteObject(calib_trial_cam_frame_ids[i]);
				calib_trial_cam_frame_ids.clear();
			}

			if (calib_toggle && trk_info.is_detected_rscam && g_info.otrk_data.calib_3d_pts.size() > 0)
			{
				// calibration routine
				Mat viewGray;
				cvtColor(imagebgr, viewGray, COLOR_BGR2GRAY);

				std::vector<__MarkerDetInfo> list_det_armks;
				ar_marker.track_markers(list_det_armks, viewGray.data, viewGray.cols, viewGray.rows, mk_ids);

				for (int i = 0; i < (int)list_det_armks.size(); i++)
				{
					__MarkerDetInfo& armk = list_det_armks[i];

					int id = armk.id;
					glm::fvec3 _rgb = marker_color(id - 1, g_info.otrk_data.calib_3d_pts.size() / 2);

					for (int j = 0; j < 4; j++)
						circle(imagebgr, Point(armk.corners2d[2 * j + 0], armk.corners2d[2 * j + 1]), 1, CV_RGB(_rgb.r * 255, _rgb.g * 255, _rgb.b * 255), 2);
				}

				if (list_det_armks.size() > 0)
				{
					vector<Point2f> point2d;
					vector<Point3f> point3d;

					for (int i = 0; i < (int)list_det_armks.size(); i++)
					{
						__MarkerDetInfo& armk = list_det_armks[i];
						if (armk.id > g_info.otrk_data.calib_3d_pts.size()) continue;

						Point2f pt2d = Point2f(0, 0);
						vector<float>& cpts = armk.corners2d;
						for (int k = 0; k < 4; k++)
							pt2d += Point2f(cpts[k * 2 + 0], cpts[k * 2 + 1]);

						point2d.push_back(pt2d / 4.f);
						point3d.push_back(g_info.otrk_data.calib_3d_pts[armk.id - 1]);
					}

					static glm::fmat4x4 prev_mat_clf2ws;
					glm::fvec3 diff = tr_pt(mat_clf2ws, glm::fvec3()) - tr_pt(prev_mat_clf2ws, glm::fvec3());

					if (do_initialize_trk_points || (glm::length(diff) > 0.05 && point2d.size() > 0))
					{
						prev_mat_clf2ws = mat_clf2ws;
						bool is_success = CalibrteCamLocalFrame(*(vector<glm::fvec2>*)&point2d, *(vector<glm::fvec3>*)&point3d, mat_ws2clf,
							rgb_intrinsics.fx, rgb_intrinsics.fy, rgb_intrinsics.ppx, rgb_intrinsics.ppy,
							do_initialize_trk_points ? CALIB_STATE::INITIALIZE : CALIB_STATE::UPDATE, mat_rscs2clf, &pnp_err, &calib_samples);
						if (is_success)
						{
							g_info.is_calib_cam = true;
							num_calib++;
						}

						for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
						{
							vzm::ObjStates cstate;
							vzm::GetSceneObjectState(g_info.ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
							cstate.is_visible = true;
							vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
						}

						int calib_frame_id = 0;
						Axis_Gen(mat_clf2ws, 0.05f, calib_frame_id);
						vzm::ObjStates cstate = obj_state;
						cstate.color[3] = 0.3f;
						vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, calib_frame_id, cstate);
						calib_trial_cam_frame_ids.push_back(calib_frame_id);
					}
					do_initialize_trk_points = false;
				}
			}
			if (!show_calib_frames)
			{
				for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
				{
					vzm::ObjStates cstate;
					vzm::GetSceneObjectState(g_info.ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
					cstate.is_visible = false;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
				}
			}
			if (show_pc)
			{
				vzm::ObjStates obj_state_pts = obj_state;
				obj_state_pts.color[3] = 1.f;
				obj_state_pts.emission = 0.3f;
				obj_state_pts.diffusion = 1.f;
				obj_state_pts.point_thickness = 0;// 2.f;
				// Generate the pointcloud and texture mappings
				pc.map_to(color); // before pc.calculate, which generates texture_coordinates
				points = pc.calculate(depth_frame);
				if (points)
				{
					auto vertices = points.get_vertices();              // get vertices
					auto tex_coords = points.get_texture_coordinates(); // and texture coordinates

					vzm::CameraParameters _cam_params;
					vzm::GetCameraParameters(g_info.ws_scene_id, _cam_params, ov_cam_id);

					//vector<glm::fvec3> vtx;// (points.size());
					//memcpy(&vtx[0], &vertices[0], sizeof(glm::fvec3) * points.size());
					glm::fmat4x4 mat_r = glm::rotate(-glm::pi<float>(), glm::fvec3(1, 0, 0));
					glm::fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;
					glm::fmat4x4 mat_os2ws;


					{
						glm::fmat4x4 mat_rs2ws = mat_rscs2ws * mat_r;
						rs2_extrinsics rgb_extrinsics = stream_depth.get_extrinsics_to(stream_rgb);
						const float *rv = rgb_extrinsics.rotation;
						glm::fmat4x4 mat_rt(rv[0], rv[1], rv[2], 0, rv[3], rv[4], rv[5], 0, rv[6], rv[7], rv[8], 0,
							rgb_extrinsics.translation[0], rgb_extrinsics.translation[1], rgb_extrinsics.translation[2], 1); // ignore 4th row 
						//*(glm::fmat4x4*) obj_state_pts.os2ws = mat_rs2ws * mat_rt;
						mat_os2ws = mat_rs2ws * mat_rt; // depth to rgb (external)
					}
					glm::fvec3* normalmap = NULL;
					const int _w = depth_frame.as<rs2::video_frame>().get_width();
					const int _h = depth_frame.as<rs2::video_frame>().get_height();
					{
						// compute face normal
						normalmap = new glm::fvec3[_w * _h];
						auto depth_color = depth_frame.apply_filter(color_map);

						// 
						glm::fmat4x4 mat_irss2os = mat_r * mat_irss2ircs;
						glm::fmat4x4 mat_irss2ws = mat_os2ws * mat_irss2os;
						glm::fvec3 pos_ir_cam_ws = tr_pt(mat_os2ws, glm::fvec3(0));
						auto ComputePos_SSZ2WS = [](const int x, const int y, const float z, const glm::fvec3& pos_ir_cam_ws, const glm::fmat4x4& mat_irss2ws)
						{
							// g_cbCamState.mat_ss2ws
							// g_cbCamState.dir_view_ws
							// g_cbCamState.pos_cam_ws
							// g_cbCamState.cam_flag & 0x1d
							// g_cbCamState.rt_width
							// deep_k_buf
							glm::fvec3 pos_ip_ws = tr_pt(mat_irss2ws, glm::fvec3(x, y, 0));
							// always perspective (do not consider orthogonal viewing)
							glm::fvec3 view_dir_ws = pos_ip_ws - pos_ir_cam_ws;

							view_dir_ws = normalize(view_dir_ws);
							return pos_ip_ws + view_dir_ws * z;
						};

						//int z_valid_count = 0;
						//float zmin = 1000000;
						//float zmax = 0;
						for (int i = 0; i < _h; i++)
							for (int j = 0; j < _w; j++)
							{
								float z = depth_frame.get_distance(j, i);
								//if (z > 0 && z < 100) 
								//	z_valid_count++;
								float z_dxR = depth_frame.get_distance(min(j + 1, _w - 1), i);
								float z_dxL = depth_frame.get_distance(max(j - 1, 0), i);
								float z_dyR = depth_frame.get_distance(j, min(i + 1, _h - 1));
								float z_dyL = depth_frame.get_distance(j, max(i - 1, 0));
								float zRx_diff = z - z_dxR;
								float zLx_diff = z - z_dxL;
								float zRy_diff = z - z_dyR;
								float zLy_diff = z - z_dyL;
								float z_dx = z_dxR, z_dy = z_dyR;
								float x_offset = 1, y_offset = 1;
								if (zRx_diff*zRx_diff > zLx_diff*zLx_diff)
								{
									z_dx = z_dxL;
									x_offset *= -1;
								}
								if (zRy_diff*zRy_diff > zLy_diff*zLy_diff)
								{
									z_dy = z_dyL;
									y_offset *= -1;
								}
								glm::fvec3 p = ComputePos_SSZ2WS(j, i, z, pos_ir_cam_ws, mat_irss2ws);
								glm::fvec3 p_dx = ComputePos_SSZ2WS(j + x_offset, i, z_dx, pos_ir_cam_ws, mat_irss2ws);
								glm::fvec3 p_dy = ComputePos_SSZ2WS(j, i + y_offset, z_dy, pos_ir_cam_ws, mat_irss2ws);
								glm::fvec3 p_ddx = p_dx - p;
								glm::fvec3 p_ddy = p_dy - p;
								glm::fvec3 face_normal = glm::normalize(glm::cross(p_ddx, p_ddy));
								normalmap[j + i * _w] = face_normal;
							}

						Mat image_depth(Size(_w, _h), CV_8UC3, (void*)depth_color.get_data(), Mat::AUTO_STEP);
						//Mat image_depth(Size(_w, _h), CV_16UC1, (void*)data_depth.get_data(), Mat::AUTO_STEP);
						imshow("test depth", image_depth);
						//cout << "TEST count : " << zmin << ", " << zmax << endl;
					}
					vector<glm::fvec2> tex(points.size());
					memcpy(&tex[0], &tex_coords[0], sizeof(glm::fvec2) * points.size());
					vector<glm::fvec3> color_pts(points.size());
					vector<glm::fvec3> pos_pts(points.size());
					vector<glm::fvec3> nrl_pts(points.size());
					for (int i = 0; i < (int)points.size(); i++)
					{
						float tx = tex[i].x * w;
						float ty = tex[i].y * h;
						int _tx = (int)tx;
						int _ty = (int)ty;
						if (_tx < 0 || _ty < 0 || _tx >= w || _ty >= h)
							continue;
						glm::u8vec3* _data = (glm::u8vec3*)image.data;
						glm::u8vec3 _color0 = _data[_tx + _ty * w];
						//glm::u8vec3 _color1 = _data[_tx + _ty * w];
						//glm::u8vec3 _color2 = _data[_tx + _ty * w];
						//glm::u8vec3 _color3 = _data[_tx + _ty * w];
						color_pts[i] = glm::fvec3(_color0.x / 255.f, _color0.y / 255.f, 1.f);// _color0.z / 255.f);
						pos_pts[i] = tr_pt(mat_os2ws, *(glm::fvec3*)&vertices[i]);
						if (normalmap) nrl_pts[i] = normalmap[i % _w + (i / _w) * _w];
					}
					if (normalmap) delete[] normalmap;
					//vzm::GeneratePointCloudObject(__FP pos_pts[0], NULL, __FP color_pts[0], (int)points.size(), g_info.rs_pc_id);
					vzm::GeneratePointCloudObject(__FP pos_pts[0], __FP nrl_pts[0], NULL, (int)points.size(), g_info.rs_pc_id);
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.rs_pc_id, obj_state_pts);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.rs_pc_id, obj_state_pts);
					vzm::DebugTestSet("_bool_OnlyForemostSurfaces", &foremost_surf_rendering, sizeof(bool), g_info.ws_scene_id, ov_cam_id, g_info.rs_pc_id);
				}
			}
			else
			{
				vzm::ObjStates obj_state_pts;
				vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.rs_pc_id, obj_state_pts);
				obj_state_pts.is_visible = false;
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.rs_pc_id, obj_state_pts);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.rs_pc_id, obj_state_pts);
			}

			if (g_info.is_calib_cam)
			{
				g_info.pos_probe_pin = tr_pt(trk_info.mat_probe2ws, glm::fvec3());
				glm::fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;

				//rs_cam_tris_id, rs_cam_lines_id, rs_cam_txt_id
				if (trk_info.is_detected_rscam)
					Register_CamModel(g_info.ws_scene_id, mat_rscs2ws, "RS CAM 0", 2);

				auto register_mks = [&obj_state](const glm::fvec3* pos_list, const int num_mks, const float r, const bool is_show)
				{
					static int obj_mks_id = 0;
					vector<glm::fvec4> sphers_xyzr;
					vector<glm::fvec3> sphers_rgb;
					for (int i = 0; i < num_mks; i++)
					{
						glm::fvec3 pt = pos_list[i];
						sphers_xyzr.push_back(glm::fvec4(pt.x, pt.y, pt.z, r));
						sphers_rgb.push_back(glm::fvec3(1, 1, 0));
					}

					if (num_mks > 0 && is_show)
					{
						vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], num_mks, obj_mks_id);
						vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_mks_id, obj_state);
						vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, obj_mks_id, obj_state);
					}
					else
					{
						vzm::ObjStates cstate = obj_state;
						cstate.is_visible = false;
						vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_mks_id, cstate);
						vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, obj_mks_id, cstate);
					}
				};
				register_mks((glm::fvec3*)&trk_info.mk_xyz_list[0], trk_info.mk_xyz_list.size() / 3, 0.005, show_mks);
			}

			static vector<int> mk_pickable_sphere_ids;
			if (g_info.rs_ms_mode == RsMouseMode::ADD_CALIB_POINTS)
			{
				auto marker_color_B = [](int idx, int w)
				{
					return glm::fvec3(1, (idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1));
				};

				int num_mks = trk_info.mk_xyz_list.size() / 3;
				g_info.vzmobjid2mkid.clear();
				for (int i = 0; i < (int)mk_pickable_sphere_ids.size(); i++)
					vzm::DeleteObject(mk_pickable_sphere_ids[i]);

				mk_pickable_sphere_ids.clear();
				for (int i = 0; i < num_mks; i++)
				{
					mk_pickable_sphere_ids.push_back(0);
					glm::fvec3 pt = trk_info.GetMkPos(i);
					vzm::GenerateSpheresObject(__FP glm::fvec4(pt.x, pt.y, pt.z, 0.01), __FP marker_color_B(i, 7), 1, mk_pickable_sphere_ids[i]);
					g_info.vzmobjid2mkid[mk_pickable_sphere_ids[i]] = pt;
					vzm::ValidatePickTarget(mk_pickable_sphere_ids[i]);
					vzm::ObjStates cstate = obj_state;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, mk_pickable_sphere_ids[i], cstate);
					//vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, mk_pickable_sphere_ids[i], cstate);
				}
			}
			else
			{
				for (int i = 0; i < (int)mk_pickable_sphere_ids.size(); i++)
					vzm::DeleteObject(mk_pickable_sphere_ids[i]);
				mk_pickable_sphere_ids.clear();
				g_info.vzmobjid2mkid.clear();
			}


			//if (trk_info.mk_residue_list.size() > 5)
			//{
			//	for (int i = 0; i < trk_info.mk_residue_list.size(); i++)
			//	{
			//		//cout << i << " ==> " << trk_info.mk_residue_list[i] << endl;
			//	}
			//}

			auto set_rb_axis = [&obj_state](const bool is_detected, const glm::fmat4x4& mat_frm2ws, int& obj_id)
			{
				if (is_detected)
				{
					Axis_Gen(mat_frm2ws, 0.07, obj_id);
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_id, obj_state);
				}
				else if (obj_id != 0)
				{
					vzm::ObjStates ostate = obj_state;
					ostate.is_visible = false;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_id, ostate);
				}
			};
			set_rb_axis(trk_info.is_detected_rscam, trk_info.mat_rbcam2ws, rs_lf_axis);
			set_rb_axis(trk_info.is_detected_probe, trk_info.mat_probe2ws, probe_lf_axis);
			set_rb_axis(trk_info.is_detected_sstool, trk_info.mat_tfrm2ws, sstool_lf_axis);

			if (trk_info.is_detected_sstool)
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

					vzm::ObjStates cobjstate = obj_state;
					*(glm::fmat4x4*) cobjstate.os2ws = trk_info.mat_tfrm2ws;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
				}
			}

			if (trk_info.is_detected_sshead)
			{
				static glm::fmat4x4 mat_os2headfrm;
				vzm::ObjStates model_obj_state, brain_obj_state, ventricle_obj_state;
				vzm::GetSceneObjectState(g_info.model_scene_id, g_info.model_obj_id, model_obj_state);
				vzm::GetSceneObjectState(g_info.model_scene_id, g_info.brain_obj_id, brain_obj_state);
				vzm::GetSceneObjectState(g_info.model_scene_id, g_info.ventricle_obj_id, ventricle_obj_state);

				model_obj_state.color[3] = 0.5;
				brain_obj_state.color[3] = 0.7;
				ventricle_obj_state.color[3] = 1.0;

				if (g_info.align_matching_model)
				{
					cout << "register rigid model!" << endl;
					glm::fmat4x4 mat_ws2headfrm = glm::inverse(trk_info.mat_headfrm2ws);
					mat_os2headfrm = mat_ws2headfrm * g_info.mat_match_model2ws;
					g_info.align_matching_model = false;
				}

				__cm4__ model_obj_state.os2ws = trk_info.mat_headfrm2ws * mat_os2headfrm;
				__cm4__ brain_obj_state.os2ws = trk_info.mat_headfrm2ws * mat_os2headfrm;
				__cm4__ ventricle_obj_state.os2ws = trk_info.mat_headfrm2ws * mat_os2headfrm;

				// REFACTORING 필요!!!!
				//SetTransformMatrixOS2WS 을 SCENE PARAM 으로 바꾸기!
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, model_obj_ws_id, model_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, model_obj_ws_id, model_obj_state);

				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.brain_obj_id, brain_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.brain_obj_id, brain_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.ventricle_obj_id, ventricle_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.ventricle_obj_id, ventricle_obj_state);

				// PIN REF //
				bool is_section_probe_detected = trk_info.is_detected_probe;
				glm::fmat4x4 mat_section_probe2ws = trk_info.mat_probe2ws;
				static int section_probe_line_id = 0, section_probe_end_id = 0;
				if (is_section_probe_detected)
				{
					glm::fvec3 probe_end = tr_pt(mat_section_probe2ws, glm::fvec3(0));
					glm::fvec3 probe_dir = glm::normalize(tr_vec(mat_section_probe2ws, glm::fvec3(0, 0, -1)));

					//
					int iToolIdx = -1;
					for (int i = 0, ni = s.rigidBodies.size(); i < ni; i++) {
						if (s.rigidBodies[i]->getType() == CiRigidBody::bodyType::TOOL) {
							iToolIdx = i;
							break;
						}
					}

					glm::fvec3 p1 = probe_end;
					glm::fvec3 p2 = probe_end - probe_dir * 0.2f;

					s.rigidBodies[iToolIdx]->m_visFiducialPoint[0] = btVector3(p1.x, p1.y, p1.z);
					s.rigidBodies[iToolIdx]->m_visFiducialPoint[1] = btVector3(p2.x, p2.y, p2.z);


					if (show_csection)
					{
						vzm::ReplaceOrAddSceneObject(g_info.csection_scene_id, model_obj_ws_id, model_obj_state);

						vzm::CameraParameters csection_cam_params_model = cam_params;
						csection_cam_params_model.np = 0.0f;
						csection_cam_params_model.fp = 10.0f;
						csection_cam_params_model.projection_mode = 4;
						csection_cam_params_model.ip_w = 0.1;
						csection_cam_params_model.ip_h = 0.1;
						csection_cam_params_model.w = 512;
						csection_cam_params_model.h = 512;

						__cv3__ csection_cam_params_model.pos = probe_end;
						glm::fvec3 cs_up = tr_vec(mat_section_probe2ws, glm::fvec3(0, 0, -1));
						glm::fvec3 cs_view = glm::fvec3(0, 0, 1);

						glm::fvec3 cs_right = glm::cross(cs_view, cs_up);
						cs_up = glm::normalize(glm::cross(cs_up, cs_right));
						__cv3__ csection_cam_params_model.up = cs_up;
						__cv3__ csection_cam_params_model.view = cs_view;
						vzm::SetCameraParameters(g_info.csection_scene_id, csection_cam_params_model, 0);

						cs_up = tr_vec(mat_section_probe2ws, glm::fvec3(0, 0, -1));
						cs_view = glm::fvec3(1, 0, 0);

						cs_right = glm::cross(cs_view, cs_up);
						cs_up = glm::normalize(glm::cross(cs_up, cs_right));
						__cv3__ csection_cam_params_model.up = cs_up;
						__cv3__ csection_cam_params_model.view = cs_view;
						vzm::SetCameraParameters(g_info.csection_scene_id, csection_cam_params_model, 1);

						vzm::RenderScene(g_info.csection_scene_id, 0);
						vzm::RenderScene(g_info.csection_scene_id, 1);
						unsigned char* cs_ptr_rgba[2];
						float* cs_ptr_zdepth[2];
						int cs_w[2], cs_h[2];
						for (int i = 0; i < 2; i++)
						{
							vzm::GetRenderBufferPtrs(g_info.csection_scene_id, &cs_ptr_rgba[i], &cs_ptr_zdepth[i], &cs_w[i], &cs_h[i], 0);
							cv::Mat cs_cvmat(cs_h[i], cs_w[i], CV_8UC4, cs_ptr_rgba[i]);
							cv::line(cs_cvmat, cv::Point(cs_w[i] / 2, cs_h[i] / 2), cv::Point(cs_w[i] / 2, 0), cv::Scalar(255, 255, 0), 2, 2);
							cv::circle(cs_cvmat, cv::Point(cs_w[i] / 2, cs_h[i] / 2), 2, cv::Scalar(255, 0, 0), 2);
							cv::imshow("Sectional View " + to_string(i), cs_cvmat);
						}
					}

					glm::fvec3 cyl_p01[2] = { probe_end, probe_end - probe_dir * 0.2f };
					float cyl_r = 0.003f;
					glm::fvec3 cyl_rgb = glm::fvec3(0, 1, 1);
					vzm::GenerateCylindersObject((float*)cyl_p01, &cyl_r, __FP cyl_rgb, 1, section_probe_line_id);
					vzm::GenerateSpheresObject(__FP glm::fvec4(probe_end, 0.0045f), __FP glm::fvec3(1, 1, 1), 1, section_probe_end_id);

					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_line_id, obj_state);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_line_id, obj_state);
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_end_id, obj_state);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_end_id, obj_state);
				}
				else
				{
					vzm::ObjStates cobj_state = obj_state;
					cobj_state.is_visible = false;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_line_id, cobj_state);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_line_id, cobj_state);
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_end_id, cobj_state);
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_end_id, cobj_state);
				}

				
			}

			vzm::CameraParameters _rs_cam_params;
			vzm::GetCameraParameters(g_info.rs_scene_id, _rs_cam_params, rs_cam_id);
			ComputeCameraStates(mat_rscs2clf, mat_clf2ws, _rs_cam_params);
			vzm::SetCameraParameters(g_info.rs_scene_id, _rs_cam_params, rs_cam_id);

			scn_env_params.is_on_camera = false;
			__cv3__ scn_env_params.pos_light = __cv3__ _rs_cam_params.pos;
			__cv3__ scn_env_params.dir_light = __cv3__ _rs_cam_params.view;
			vzm::SetSceneEnvParameters(g_info.ws_scene_id, scn_env_params);

			vzm::RenderScene(g_info.rs_scene_id, rs_cam_id);
			unsigned char* ptr_rgba;
			float* ptr_zdepth;
			int rs_w, rs_h;
			if (vzm::GetRenderBufferPtrs(g_info.rs_scene_id, &ptr_rgba, &ptr_zdepth, &rs_w, &rs_h, rs_cam_id))
				copy_back_ui_buffer(imagebgr.data, ptr_rgba, rs_w, rs_h, false);

			cv::putText(imagebgr, "PnP reprojection error : " + to_string(pnp_err) + " pixels, # samples : " + to_string(calib_samples),
				cv::Point(3, 25), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			cv::putText(imagebgr, "Calibration Points : " + to_string(g_info.otrk_data.calib_3d_pts.size()),
				cv::Point(3, 50), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(185, 255, 255));
			cv::putText(imagebgr, "# of calibrations : " + to_string(num_calib),
				cv::Point(3, 75), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			cv::putText(imagebgr, "mouse mode : " + EtoString(g_info.rs_ms_mode), cv::Point(3, 150), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			string b_calib_toggle = calib_toggle ? "true" : "false";
			cv::putText(imagebgr, "Calibration Toggle : " + b_calib_toggle + ", Postpone : " + to_string(postpone) + " ms",
				cv::Point(3, 100), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));

			if (g_info.is_calib_cam && !trk_info.is_detected_rscam)
				cv::putText(imagebgr, "RS Cam is out of tracking volume !!", cv::Point(400, 50), cv::FONT_HERSHEY_DUPLEX, 2.0, CV_RGB(255, 0, 0), 3, LineTypes::LINE_AA);

			imshow(g_info.window_name_rs_view, imagebgr);
			/**/
		}

		vzm::DebugTestSet("_bool_ReloadHLSLObjFiles", &recompile_hlsl, sizeof(bool), -1, -1);
		vzm::DebugTestSet("_bool_TestOit", &use_new_version, sizeof(bool), -1, -1);
		Show_Window(g_info.window_name_ws_view, g_info.ws_scene_id, ov_cam_id);
		switch (key_pressed)
		{
		case 100: recompile_hlsl = false; break;
		}
	}

	// Signal threads to finish and wait until they do
	rs_alive = false;
	video_processing_thread.join();
	tracker_alive = false;
	tracker_processing_thread.join();
	optitrk::DeinitOptiTrackLib();

	vzm::DeinitEngineLib();

	return 0;
}