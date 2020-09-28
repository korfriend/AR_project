#include "ArSettings.h"
#include "../optitrk/optitrack.h"
#include "../aruco_marker/aruco_armarker.h"

#include <string>
#include <map>
#include <thread>
#include <atomic>
#include <mutex>
#include "VisMtvApi.h"

using namespace std;
using namespace cv;
#include "../kar_helpers.hpp"
#include "../event_handler.hpp"

#define __MIRRORS

namespace rs_settings
{
	map<string, string> serials;
	rs2::decimation_filter _dec;
	rs2::disparity_transform depth2disparity;
	rs2::disparity_transform disparity2depth(false);
	rs2::spatial_filter spat;
	rs2::temporal_filter temp;
	// Spatially align all streams to depth viewport
	// We do this because:
	//   a. Usually depth has wider FOV, and we only really need depth for this demo
	//   b. We don't want to introduce new holes
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::align align_to(RS2_STREAM_DEPTH);
	rs2_intrinsics rgb_intrinsics;
	rs2_intrinsics depth_intrinsics;
	rs2_extrinsics rgb_extrinsics;

	bool is_initialized = false;
	bool _use_depthsensor = false;
	bool _use_testeyecam = false;

	// Alive boolean will signal the worker threads to finish-up
	std::atomic_bool rs_alive{ false };
	std::atomic_bool eye_rs_alive{ false };
	std::thread video_processing_thread;
	std::thread eye_processing_thread;

	rs2::context* _ctx;
	rs2::pipeline* _pipe;// (ctx);
	rs2::pipeline* _eye_pipe;// (ctx);

	// Colorizer is used to visualize depth data
	rs2::colorizer color_map;
	// Declare pointcloud object, for calculating pointclouds and texture mappings
	rs2::pointcloud pc;
	// We want the points object to be persistent so we can display the last cloud when a frame drops
	rs2::points points;

	glm::fmat4x4 mat_ircs2irss, mat_irss2ircs;

	void GetRsCamParams(rs2_intrinsics& _rgb_intrinsics, rs2_intrinsics& _depth_intrinsics, rs2_extrinsics& _rgb_extrinsics)
	{
		_rgb_intrinsics = rgb_intrinsics;
		_depth_intrinsics = depth_intrinsics;
		_rgb_extrinsics = rgb_extrinsics;
	}

	void InitializeRealsense(const bool use_depthsensor, const bool use_testeyecam,
		const int rs_w, const int rs_h, const int eye_w, const int eye_h)
	{
		if (is_initialized) return;

		_use_depthsensor = use_depthsensor;
		_use_testeyecam = use_testeyecam;

		//for (auto&& dev : ctx.query_devices())
		//	serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
		//cout << serials[0] << endl; // 839112061828 // 415
		//cout << serials[1] << endl; // 819312071259 // 430
		serials["RS_RBS"] = string("839112061828");
		serials["EYE"] = string("819312071259");
		//serials.insert(std::pair<std::string, std::string>("RS_RBS", "819312071259"));
		//serials.insert(std::pair<std::string, std::string>("EYE", "819312071259"));


		// Decimation filter reduces the amount of data (while preserving best samples)
		// If the demo is too slow, make sure you run in Release (-DCMAKE_BUILD_TYPE=Release)
		// but you can also increase the following parameter to decimate depth more (reducing quality)
		_dec.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
		// Define spatial filter (edge-preserving)
		// Enable hole-filling
		// Hole filling is an agressive heuristic and it gets the depth wrong many times
		// However, this demo is not built to handle holes
		// (the shortest-path will always prefer to "cut" through the holes since they have zero 3D distance)
		spat.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5);
		spat.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 1.0);
		spat.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2);
		spat.set_option(RS2_OPTION_HOLES_FILL, 0); // 5 = fill all the zero pixels
		// Define temporal filter
		temp.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0);
		temp.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 100.0);

		//rs2::context ctx;
		//rs2::pipeline pipe(ctx);
		//rs2::pipeline eye_pipe(ctx);

		// Use black to white color map
		color_map.set_option(RS2_OPTION_COLOR_SCHEME, 2.f);

		_ctx = new rs2::context();
		rs2::context& ctx = *_ctx;
		_pipe = new rs2::pipeline(ctx);
		_eye_pipe = new rs2::pipeline(ctx);

		rs2::pipeline& pipe = *_pipe;
		rs2::pipeline& eye_pipe = *_eye_pipe;

		rs2::config cfg;
		rs2::config eye_cfg;

		//_pipe = &pipe;
		//_eye_pipe = &eye_pipe;

		cfg.enable_device(serials["RS_RBS"]);
		cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60); // Enable default depth

		if (!use_depthsensor)
			cfg.disable_stream(RS2_STREAM_DEPTH); // Disable default depth

		// For the color stream, set format to RGBA
		// To allow blending of the color frame on top of the depth frame
		cfg.enable_stream(RS2_STREAM_COLOR, 0, rs_w, rs_h, RS2_FORMAT_RGB8, 60);

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


		if (use_depthsensor)
		{
			auto stream_depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
			//rs2_extrinsics rgb_extrinsics = stream_depth.get_extrinsics_to(stream_rgb);
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

				rgb_extrinsics = stream_depth.get_extrinsics_to(stream_rgb);
			}
		}

		if (use_testeyecam)
		{
			eye_cfg.enable_device(serials["EYE"]);
			eye_cfg.disable_stream(RS2_STREAM_DEPTH); // Disable default depth
			// For the color stream, set format to RGBA
			// To allow blending of the color frame on top of the depth frame
			eye_cfg.enable_stream(RS2_STREAM_COLOR, 0, eye_w, eye_h, RS2_FORMAT_RGB8, 60);
			eye_pipe.start(eye_cfg);
		}
		is_initialized = true;
	}

	void RunRsThread(rs2::frame_queue& original_data, rs2::frame_queue& filtered_data, rs2::frame_queue& eye_data)
	{
		if (!is_initialized | rs_alive) return;

		rs_alive = true;
		video_processing_thread = std::thread([&]() {
			while (rs_alive)
			{
				// Fetch frames from the pipeline and send them for processing
				//rs2::frameset data;
				//if (pipe.poll_for_frames(&data))
				{
					rs2::frameset data = _pipe->wait_for_frames(); // Wait for next set of frames from the camera
					rs2::frame data_depth = data.get_depth_frame();

					if (data_depth != NULL && _use_depthsensor)
					{
						// First make the frames spatially aligned
						data_depth = data_depth.apply_filter(align_to);

						// Decimation will reduce the resultion of the depth image,
						// closing small holes and speeding-up the algorithm
						data_depth = data_depth.apply_filter(_dec);

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
					}

					filtered_data.enqueue(data_depth);
					original_data.enqueue(data);

					// Send resulting frames for visualization in the main thread
					//original_data.enqueue(data);
				}
			}
		});

		if (_use_testeyecam)
		{
			eye_rs_alive = true;
			eye_processing_thread = std::thread([&]() {
				while (eye_rs_alive)
				{
					// Fetch frames from the pipeline and send them for processing
					{
						//rs2::frameset data = eye_pipe.wait_for_frames(); // Wait for next set of frames from the camera
						//eye_data.enqueue(data);
					}
					rs2::frameset data;
					if (_eye_pipe->poll_for_frames(&data))
					{
						eye_data.enqueue(data);
					}
				}
			});
		}
	}

	void FinishRsThreads()
	{
		rs_alive = false;
		video_processing_thread.join();
		if (_use_testeyecam)
		{
			eye_rs_alive = false;
			eye_processing_thread.join();
		}
	}

	void DeinitializeRealsense()
	{
		delete _ctx;
		delete _pipe;
		delete _eye_pipe;
		_ctx = NULL;
		_pipe = NULL;
		_eye_pipe = NULL;
		is_initialized = false;

	}
}


namespace var_settings
{
#define __NUMMARKERS 15
	GlobalInfo g_info;
	ArMarkerTracker ar_marker;
	set<int> mk_ids;
	string preset_path;
	string GetDefaultFilePath() {
		return preset_path;
	}

	int scenario = 0;

	void InitializeVarSettings(int _scenario)
	{
		scenario = _scenario;

		// set global information
		g_info.ws_scene_id = 1;
		g_info.rs_scene_id = 2;
		g_info.model_scene_id = 3;
		g_info.csection_scene_id = 4;
		g_info.stg_scene_id = 5;

		g_info.window_name_rs_view = "RealSense VIEW";
		g_info.window_name_ws_view = "World VIEW";
		g_info.window_name_ms_view = "Model VIEW";
		g_info.window_name_stg_view = "STG VIEW";

		char ownPth[2048];
		GetModuleFileNameA(NULL, ownPth, (sizeof(ownPth)));
		string exe_path = ownPth;
		size_t pos = 0;
		std::string token;
		string delimiter = "\\";
		preset_path = "";
		while ((pos = exe_path.find(delimiter)) != std::string::npos) {
			token = exe_path.substr(0, pos);
			if (token.find(".exe") != std::string::npos) break;
			preset_path += token + "\\";
			exe_path.erase(0, pos + delimiter.length());
		}
		preset_path += "..\\";
		//cout << hlslobj_path << endl;


		// load txt file
		/*
		g_info.optrack_calib = "D:\\Document\\OptiTrack\\my_test_200812_1.cal";
		g_info.optrack_env = "D:\\Document\\OptiTrack\\my_test_200812_1.motive";
		g_info.cb_positions = "E:\\project_srcs\\kar\\prototype_ver1\\cb_points.txt";
		g_info.sst_positions = "E:\\project_srcs\\kar\\prototype_ver1\\ss_pin_pts.txt";
		g_info.rs_calib = "E:\\project_srcs\\kar\\prototype_ver1\\rs_calib.txt";
		g_info.stg_calib = "E:\\project_srcs\\kar\\prototype_ver1\\stg_calib.txt";
		g_info.model_predefined_pts = "E:\\project_srcs\\kar\\prototype_ver1\\mode_predefined_points.txt";
		*/

		//	~200907
		/*
		g_info.optrack_calib = "C:\\Users\\User\\Desktop\\Preset\\Optitrack\\Calibration_200904.cal";
		g_info.optrack_env = "C:\\Users\\User\\Desktop\\Preset\\Optitrack\\Asset_200904.motive";
		g_info.cb_positions = "E:\\project_srcs\\kar\\prototype_ver1\\cb_points.txt";
		g_info.sst_positions = "E:\\project_srcs\\kar\\prototype_ver1\\ss_pin_pts.txt";
		g_info.rs_calib = "E:\\project_srcs\\kar\\prototype_ver1\\rs_calib.txt";
		g_info.stg_calib = "E:\\project_srcs\\kar\\prototype_ver1\\stg_calib.txt";
		g_info.model_predefined_pts = "E:\\project_srcs\\kar\\prototype_ver1\\mode_predefined_points.txt";
		*/
		
		g_info.optrack_calib = preset_path + "..\\Preset\\CalibrationResult 2020-09-22 9.cal";
		//g_info.optrack_env = preset_path + "..\\Preset\\Asset_200911.motive";
		g_info.optrack_env = preset_path + "..\\Preset\\Motive Profile - 2020-09-22.motive";
		g_info.cb_positions = preset_path + "..\\Preset\\cb_points.txt";
		g_info.sst_positions = preset_path + "..\\Preset\\ss_pin_pts.txt";
		g_info.rs_calib = preset_path + "..\\Preset\\rs_calib.txt";
		g_info.stg_calib = preset_path + "..\\Preset\\stg_calib.txt";

		if (scenario == 0)
		{
			g_info.model_path = preset_path + "..\\Data\\skin.obj";
			g_info.model_predefined_pts = preset_path + "..\\Preset\\mode_predefined_points.txt";
		}
		else if (scenario == 1)
		{
			g_info.model_path = preset_path + "..\\Data\\breast\\chest_front_points(nrl)_simple1.ply";
			g_info.volume_model_path = preset_path + "..\\Data\\breast\\chest_x3d.x3d";
			g_info.model_predefined_pts = preset_path + "..\\Preset\\mode_predefined_points(breast).txt";
		}

		for (int i = 1; i <= __NUMMARKERS; i++)
		{
			mk_ids.insert(i);
			ar_marker.register_marker(i, 5.15);
			//ar_marker.aruco_marker_file_out(i, "armk" + to_string(i) + ".bmp");
		}
	}
	
	bool _show_sectional_views = false;
	int ov_cam_id = 1; // arbitrary integer
	int model_cam_id = 1; // arbitrary integer
	int rs_cam_id = 1; // arbitrary integer
	int stg_cam_id = 1; // arbitrary integer
	int zoom_cam_id = 1; // arbitrary integer
	vzm::ObjStates default_obj_state;

	bool is_rsrb_detected = false;
	glm::fmat4x4 mat_ws2clf, mat_clf2ws;
	bool is_probe_detected = false;
	glm::fmat4x4 mat_probe2ws;

	glm::fmat4x4 mat_rscs2clf;
	glm::fmat4x4 mat_stgcs2clf;

	// rs calib history
	vector<track_info> record_trk_info;
	vector<void*> record_rsimg;
	map<string, int> action_info;
	vector<int> record_key;

	void SetPreoperations(const int rs_w, const int rs_h, const int ws_w, const int ws_h, const int stg_w, const int stg_h, const int eye_w, const int eye_h)
	{
		//printf("%s", g_info.optrack_env.c_str());
		optitrk::LoadProfileAndCalibInfo(g_info.optrack_env, g_info.optrack_calib);
		cout << "cam0 frame rate setting ==> " << optitrk::SetCameraFrameRate(0, 120) << endl;
		cout << "cam1 frame rate setting ==> " << optitrk::SetCameraFrameRate(1, 120) << endl;

		g_info.eye_w = eye_w;
		g_info.eye_h = eye_h;
		g_info.ws_w = ws_w;
		g_info.ws_h = ws_h;
		g_info.stg_w = stg_w;
		g_info.stg_h = stg_h;
		g_info.rs_w = rs_w;
		g_info.rs_h = rs_h;

		int volume_obj_id = 0;
		if (scenario == 0)
		{
			vzm::LoadModelFile(g_info.model_path, g_info.model_ms_obj_id);
			vzm::GenerateCopiedObject(g_info.model_ms_obj_id, g_info.model_ws_obj_id);
		}
		else if (scenario == 1)
		{
			vzm::LoadModelFile(g_info.model_path, g_info.model_ms_obj_id);
			vzm::GenerateCopiedObject(g_info.model_ms_obj_id, g_info.model_ws_obj_id);
			vzm::LoadModelFile(g_info.volume_model_path, g_info.model_volume_id);
		}
		vzm::ValidatePickTarget(g_info.model_ms_obj_id);

		vzm::CameraParameters cam_params;
		__cv3__ cam_params.pos = glm::fvec3(1.0, 2.0, 1.5f);
		glm::fvec3 t_up = glm::fvec3(0, 1.f, 0);
		__cv3__ cam_params.view = glm::normalize(glm::fvec3(0, 1, 0) - __cv3__ cam_params.pos);
		glm::fvec3 t_right = glm::cross(__cv3__ cam_params.view, t_up);
		__cv3__ cam_params.up = glm::normalize(glm::cross(t_right, __cv3__ cam_params.view));
		cam_params.fov_y = 3.141592654f / 4.f;
		cam_params.aspect_ratio = (float)ws_w / (float)ws_h;
		cam_params.projection_mode = 2;
		cam_params.w = ws_w;
		cam_params.h = ws_h;
		cam_params.np = 0.1f;
		cam_params.fp = 20.0f;

		vzm::SetCameraParameters(g_info.ws_scene_id, cam_params, ov_cam_id);

		vzm::CameraParameters cam_params_model = cam_params;
		cam_params_model.np = 0.01f;
		cam_params_model.fp = 10.0f;
		if (scenario == 0)
		{
			__cv3__ cam_params_model.pos = glm::fvec3(0.3f, 0, 0);
			__cv3__ cam_params_model.up = glm::fvec3(0, 1.f, 0);
			__cv3__ cam_params_model.view = glm::fvec3(-1.f, 0, 0.f);
		}
		else if (scenario == 1)
		{
			__cv3__ cam_params_model.pos = glm::fvec3(0, -0.7f, 0);
			__cv3__ cam_params_model.up = glm::fvec3(0, 0, 1.f);
			__cv3__ cam_params_model.view = glm::fvec3(0, 1.f, 0.f);
		}
		cam_params_model.w = 400;
		cam_params_model.h = 400;

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
		vzm::SceneEnvParameters stg_scn_env_params = scn_env_params;
		vzm::SetSceneEnvParameters(g_info.stg_scene_id, stg_scn_env_params);

		vzm::ObjStates obj_state;
		obj_state.emission = 0.4f;
		obj_state.diffusion = 0.6f;
		obj_state.specular = 0.2f;
		obj_state.sp_pow = 30.f;
		__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
		__cm4__ obj_state.os2ws = glm::fmat4x4();
		default_obj_state = obj_state;

		double vz = 0.001;
		vzm::DebugTestSet("_double_VZThickness", &vz, sizeof(double), -1, -1);
		double cvz = 0.0005;
		vzm::DebugTestSet("_double_CopVZThickness", &cvz, sizeof(double), -1, -1);
		bool use_new_version = true;
		vzm::DebugTestSet("_bool_TestOit", &use_new_version, sizeof(bool), -1, -1);

		vzm::ObjStates model_state = obj_state;
		model_state.emission = 0.3f;
		model_state.diffusion = 0.5f;
		model_state.specular = 0.1f;
		model_state.color[3] = 0.8;
		double scale_factor = 0.001;
		glm::fmat4x4 mat_s = glm::scale(glm::fvec3(scale_factor));
		g_info.mat_os2matchmodefrm = __cm4__ model_state.os2ws = (__cm4__ model_state.os2ws) * mat_s;
		model_state.point_thickness = 10;
		vzm::ObjStates model_ws_state;
		if (g_info.model_volume_id != 0)
		{
			int vr_tmap_id = 0, vr_tmap_id1 = 0, mpr_tmap_id = 0;
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
			alpha_ctrs[0] = glm::fvec2(0, 17760);
			alpha_ctrs[1] = glm::fvec2(1, 21700);
			vzm::GenerateMappingTable(65537, alpha_ctrs.size(), (float*)&alpha_ctrs[0], rgb_ctrs.size(), (float*)&rgb_ctrs[0], vr_tmap_id1);

			alpha_ctrs[0] = glm::fvec2(0, 100);
			alpha_ctrs[1] = glm::fvec2(1, 30000);
			rgb_ctrs[1] = glm::fvec4(1);
			rgb_ctrs[2] = glm::fvec4(1);
			vzm::GenerateMappingTable(65537, alpha_ctrs.size(), (float*)&alpha_ctrs[0], rgb_ctrs.size(), (float*)&rgb_ctrs[0], mpr_tmap_id);

			vzm::ObjStates volume_ws_state = model_state;
			volume_ws_state.associated_obj_ids["VR_OTF"] = vr_tmap_id;
			volume_ws_state.associated_obj_ids["MPR_WINDOWING"] = mpr_tmap_id;

			volume_ws_state.associated_obj_ids["VR_OTF"] = vr_tmap_id1;
			volume_ws_state.is_visible = false;
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.model_volume_id, volume_ws_state);

			double sample_rate = 1. / scale_factor;
			vzm::DebugTestSet("_double_UserSampleRate", &sample_rate, sizeof(double), -1, -1);// g_info.model_scene_id, model_cam_id);
			bool apply_samplerate2grad = true;
			vzm::DebugTestSet("_bool_ApplySampleRateToGradient", &apply_samplerate2grad, sizeof(bool), -1, -1);//g_info.model_scene_id, model_cam_id);

			//vzm::ReplaceOrAddSceneObject(g_info.model_scene_id, g_info.model_volume_id, volume_ws_state);
		}
		vzm::ReplaceOrAddSceneObject(g_info.model_scene_id, g_info.model_ms_obj_id, model_state);

		vzm::CameraParameters rs_cam_params;
		__cv3__ rs_cam_params.pos = glm::fvec3(0);
		__cv3__ rs_cam_params.up = glm::fvec3(0, 1, 0);
		__cv3__ rs_cam_params.view = glm::fvec3(0, 0, 1);

		rs_cam_params.fx = rs_settings::rgb_intrinsics.fx;
		rs_cam_params.fy = rs_settings::rgb_intrinsics.fy;
		rs_cam_params.cx = rs_settings::rgb_intrinsics.ppx;
		rs_cam_params.cy = rs_settings::rgb_intrinsics.ppy;
		rs_cam_params.sc = 0;
		rs_cam_params.w = rs_settings::rgb_intrinsics.width;
		rs_cam_params.h = rs_settings::rgb_intrinsics.height;
		rs_cam_params.np = 0.1f;
		rs_cam_params.fp = 20.0f;
		rs_cam_params.projection_mode = 3;

		vzm::SetCameraParameters(g_info.rs_scene_id, rs_cam_params, rs_cam_id);

		Show_Window(g_info.window_name_ms_view, g_info.model_scene_id, model_cam_id);

		optitrk::UpdateFrame();
		glm::fmat4x4 mat_cam0_to_ws, mat_cam1_to_ws;
		optitrk::GetCameraLocation(0, (float*)&mat_cam0_to_ws);
		optitrk::GetCameraLocation(1, (float*)&mat_cam1_to_ws);
		Update_CamModel(g_info.ws_scene_id, mat_cam0_to_ws, "IR CAM 0", 0);
		Update_CamModel(g_info.ws_scene_id, mat_cam1_to_ws, "IR CAM 1", 1);

		// make 3d ui widgets
		GenWorldGrid(g_info.ws_scene_id, ov_cam_id);
		// touch interface buttons
		Make_Buttons(g_info.rs_w, g_info.rs_h, g_info.rs_buttons);
	}

	void SetCvWindows()
	{
//#define __DOJO_PC
#define __DEMO_PC
#ifdef __DOJO_PC
		//Create a window
		cv::namedWindow(g_info.window_name_rs_view, WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_ws_view, WINDOW_AUTOSIZE | WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_ms_view, WINDOW_AUTOSIZE | WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_stg_view, WINDOW_AUTOSIZE | WINDOW_NORMAL);

		cv::moveWindow(g_info.window_name_ws_view, 2560, 0);
		cv::moveWindow(g_info.window_name_ms_view, 2560 + 1000, 0);

		cv::moveWindow(g_info.window_name_rs_view, 2560 * 3, 0);
		cv::moveWindow(g_info.window_name_stg_view, 2560 * 2, 700);
		//cv::moveWindow(g_info.window_name_rs_view, 0 * 3, 0);
		//cv::moveWindow(g_info.window_name_stg_view, 0 * 3 + 1024, 0);

		cv::setWindowProperty(g_info.window_name_rs_view, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		//cv::setWindowProperty(g_info.window_name_stg_view, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
#endif

#ifdef __DEMO_PC
		const int display_w = 1680;
		// for demo PC
		//Create a window
		cv::namedWindow(g_info.window_name_rs_view, WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_ws_view, WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_ms_view, WINDOW_NORMAL);
		cv::namedWindow(g_info.window_name_stg_view, WINDOW_NORMAL);
#ifdef __MIRRORS
		cv::namedWindow("rs mirror", WINDOW_NORMAL);
		cv::namedWindow("stg mirror", WINDOW_NORMAL);
		cv::moveWindow("rs mirror", 40, 470);
		cv::moveWindow("stg mirror", 944, 470);
#endif

		cv::moveWindow(g_info.window_name_ws_view, 550, 0);
		cv::moveWindow(g_info.window_name_ms_view, 1180, 0);
		cv::moveWindow(g_info.window_name_rs_view, display_w, 0);
		cv::moveWindow(g_info.window_name_stg_view, display_w + 1024, 0);
		////cv::moveWindow(g_info.window_name_rs_view, 0 * 3, 0);
		////cv::moveWindow(g_info.window_name_stg_view, 0 * 3 + 1024, 0);

		cv::setWindowProperty(g_info.window_name_rs_view, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
		cv::setWindowProperty(g_info.window_name_stg_view, WND_PROP_FULLSCREEN, WINDOW_FULLSCREEN);
#endif

		// for developers
		//cv::namedWindow(g_info.window_name_rs_view, WINDOW_AUTOSIZE);
		//cv::namedWindow(g_info.window_name_ws_view, WINDOW_AUTOSIZE);
		//cv::namedWindow(g_info.window_name_ms_view, WINDOW_AUTOSIZE);
		//cv::namedWindow(g_info.window_name_stg_view, WINDOW_AUTOSIZE);

		static EventGlobalInfo rg_info_world(g_info, g_info.ws_scene_id, ov_cam_id);
		cv::setMouseCallback(g_info.window_name_ws_view, CallBackFunc_WorldMouse, &rg_info_world);
		static EventGlobalInfo rg_info_model(g_info, g_info.model_scene_id, model_cam_id);
		cv::setMouseCallback(g_info.window_name_ms_view, CallBackFunc_ModelMouse, &rg_info_model);

		static EventGlobalInfo rg_info_rs(g_info, g_info.rs_scene_id, rs_cam_id);
		cv::setMouseCallback(g_info.window_name_rs_view, CallBackFunc_RsMouse, &rg_info_rs);
		static EventGlobalInfo rg_info_stg(g_info, 0, 0);
		cv::setMouseCallback(g_info.window_name_stg_view, CallBackFunc_StgMouse, &rg_info_stg);
	}

	void LoadPresets()
	{
		// loading 3d points
		g_info.otrk_data.calib_3d_pts.clear();
		std::ifstream infile(g_info.cb_positions);
		string line;
		if (infile.is_open())
		{
			while (getline(infile, line))
			{
				std::istringstream iss(line);
				float a, b, c;
				if (!(iss >> a >> b >> c)) { break; } // error
				g_info.otrk_data.calib_3d_pts.push_back(Point3f(a, b, c));
				// process pair (a,b)
			}
			infile.close();
		}

		// loading rs calib pairs and matrix
		infile = std::ifstream(g_info.rs_calib);
		if (infile.is_open())
		{
			g_info.otrk_data.tc_calib_pt_pairs.clear();
			float* mat_data = glm::value_ptr(mat_rscs2clf);
			int line_idx = 0, line_pairs = 100000;
			while (getline(infile, line))
			{
				std::istringstream iss(line);
				if (line_idx == 0)
				{
					iss >> line_pairs;
				}
				else if (line_idx < line_pairs + 1)
				{
					glm::fvec2 p2d;
					glm::fvec3 p3d;
					iss >> p2d.x >> p2d.y >> p3d.x >> p3d.y >> p3d.z;
					g_info.otrk_data.tc_calib_pt_pairs.push_back(PAIR_MAKE(p2d, p3d));
				}
				else
				{
					iss >> mat_data[line_idx - line_pairs - 1];
				}
				// process pair (a,b)
				line_idx++;
			}
			g_info.is_calib_rs_cam = true;
			infile.close();


			// loading model point pairs
			infile = std::ifstream(g_info.model_predefined_pts);
			if (infile.is_open())
			{
				g_info.model_ms_pick_pts.clear();
				while (getline(infile, line))
				{
					std::istringstream iss(line);
					float a, b, c;
					if (!(iss >> a >> b >> c)) { break; } // error
					g_info.model_ms_pick_pts.push_back(glm::fvec3(a, b, c));
				}
				infile.close();

				if (g_info.model_ms_pick_pts.size() > 0)
				{
					vector<glm::fvec4> spheres_xyzr;
					vector<glm::fvec3> spheres_rgb;
					for (int i = 0; i < (int)g_info.model_ms_pick_pts.size(); i++)
					{
						glm::fvec4 sphere_xyzr = glm::fvec4(g_info.model_ms_pick_pts[i], 0.002);
						spheres_xyzr.push_back(sphere_xyzr);
						glm::fvec3 sphere_rgb = glm::fvec3(1, 0, 0);
						spheres_rgb.push_back(sphere_rgb);
					}
					vzm::ObjStates sobj_state;
					sobj_state.color[3] = 1.0f;
					sobj_state.emission = 0.5f;
					sobj_state.diffusion = 0.5f;
					sobj_state.specular = 0.0f;
					vzm::GenerateSpheresObject(__FP spheres_xyzr[0], __FP spheres_rgb[0], (int)g_info.model_ms_pick_pts.size(), g_info.model_ms_pick_spheres_id);
					vzm::ReplaceOrAddSceneObject(g_info.model_scene_id, g_info.model_ms_pick_spheres_id, sobj_state);
					Show_Window_with_Texts(g_info.window_name_ms_view, g_info.model_scene_id, model_cam_id, "Point : " + to_string((int)g_info.model_ms_pick_pts.size()));
				}
			}
		}

		// loading stg calib points
		infile = std::ifstream(g_info.stg_calib);
		if (infile.is_open())
		{
			g_info.otrk_data.stg_calib_pt_pairs.clear();
			int line_idx = 0, line_pairs = 100000;
			while (getline(infile, line))
			{
				std::istringstream iss(line);
				if (line_idx == 0)
				{
					iss >> line_pairs;
				}
				else if (line_idx < line_pairs + 1)
				{
					Point2f p2d;
					Point3f p3d;
					iss >> p2d.x >> p2d.y >> p3d.x >> p3d.y >> p3d.z;
					g_info.otrk_data.stg_calib_pt_pairs.push_back(pair<Point2f, Point3f>(p2d, p3d));
				}
				// process pair (a,b)
				line_idx++;
			}
			infile.close();
		}
	}

	/////////////////////////////////////////////////////////////
	int GetCameraID_SSU(const int scene_id)
	{
		// ov_cam_id
		// model_cam_id
		// rs_cam_id
		// stg_cam_id
		// zoom_cam_id
		if (scene_id == g_info.ws_scene_id) {
			return ov_cam_id;
		}
		else if (scene_id == g_info.rs_scene_id) {
			return rs_cam_id;
		}
		else if (scene_id == g_info.model_scene_id) {
			return model_cam_id;
		}
		else if (scene_id == g_info.stg_scene_id) {
			return stg_cam_id;
		}
		else if (scene_id == g_info.zoom_scene_id) {
			return zoom_cam_id;
		}

		return -1;
	}

	/////////////////////////////////////////////////////////////

	auto clear_record_info = [&]()
	{
		for (int i = 0; i < (int)record_rsimg.size(); i++)
		{
			delete[] record_rsimg[i];
		}
		record_trk_info.clear();
		record_rsimg.clear();
		action_info.clear();
		record_key.clear();
	};

	void ResetCalib()
	{
		g_info.otrk_data.calib_3d_pts.clear();
		g_info.otrk_data.stg_calib_pt_pairs.clear();
		g_info.otrk_data.tc_calib_pt_pairs.clear();
		for (int i = 0; i < g_info.otrk_data.calib_trial_rs_cam_frame_ids.size(); i++)
			vzm::DeleteObject(g_info.otrk_data.calib_trial_rs_cam_frame_ids[i]);
		g_info.otrk_data.calib_trial_rs_cam_frame_ids.clear();
		g_info.is_calib_stg_cam = g_info.is_calib_rs_cam = false;
		g_info.otrk_data.stg_calib_pt_pairs.clear();
		g_info.model_predefined_pts.clear();
		vzm::DeleteObject(g_info.model_ws_pick_spheres_id);
		cout << "CLEAR calibration points" << endl;
	}

	void StoreRecordInfo()
	{
		const int w = g_info.rs_w;
		const int h = g_info.rs_h;
		cout << "WRITE RECODING INFO of " << record_trk_info.size() << " frames" << endl;

		std::fstream file_imgdata;
		file_imgdata.open("imgdata.bin", std::ios::app | std::ios::binary);
		file_imgdata.clear();
		file_imgdata.write(reinterpret_cast<char*>(&record_rsimg[0]), w * h * 3 * sizeof(char) * record_rsimg.size());
		file_imgdata.close();

		std::fstream file_trkdata;
		file_trkdata.open("trkdata.bin", std::ios::app | std::ios::binary);
		file_trkdata.clear();
		for (int i = 0; i < (int)record_trk_info.size(); i++)
		{
			size_t buf_size_bytes;
			char* _buf = record_trk_info[i].GetSerialBuffer(buf_size_bytes);
			file_trkdata.write(_buf, buf_size_bytes);
			delete[] _buf;
		}
		file_trkdata.close();

		ofstream file_keydata("keyrecord.txt");
		if (file_keydata.is_open())
		{
			file_keydata.clear();
			file_keydata << to_string(record_key.size()) << endl;
			for (int i = 0; i < (int)record_key.size(); i++)
			{
				file_keydata << to_string(record_key[i]) << endl;
			}
		}
		file_keydata.close();

		clear_record_info();
	}

	void UpdateTrackInfo(const void* trk_info)
	{
		g_info.otrk_data.trk_info = *(track_info*)trk_info;
		is_rsrb_detected = g_info.otrk_data.trk_info.GetLFrmInfo("rs_cam", mat_clf2ws);
		mat_ws2clf = glm::inverse(mat_clf2ws);

		static int section_probe_line_id = 0, section_probe_end_id = 0;
		is_probe_detected = g_info.otrk_data.trk_info.GetLFrmInfo("probe", mat_probe2ws);
		if (is_probe_detected)
		{
			g_info.pos_probe_pin = tr_pt(mat_probe2ws, glm::fvec3(0));
			glm::fvec3 probe_end = g_info.pos_probe_pin;
			glm::fvec3 probe_dir = glm::normalize(tr_vec(mat_probe2ws, glm::fvec3(0, 0, -1)));

			glm::fvec3 cyl_p01[2] = { probe_end, probe_end - probe_dir * 0.2f };
			float cyl_r = 0.002f;
			glm::fvec3 cyl_rgb = glm::fvec3(0, 1, 1);
			vzm::GenerateCylindersObject((float*)cyl_p01, &cyl_r, __FP cyl_rgb, 1, section_probe_line_id);
			vzm::GenerateSpheresObject(__FP glm::fvec4(probe_end, 0.0045f), __FP glm::fvec3(1, 1, 1), 1, section_probe_end_id);

			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_line_id, default_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_line_id, default_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, section_probe_line_id, default_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_end_id, default_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_end_id, default_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, section_probe_end_id, default_obj_state);
		}
		else
		{
			vzm::ObjStates cobj_state = default_obj_state;
			cobj_state.is_visible = false;
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_line_id, cobj_state);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_line_id, cobj_state);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, section_probe_line_id, cobj_state);
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, section_probe_end_id, cobj_state);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, section_probe_end_id, cobj_state);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, section_probe_end_id, cobj_state);
		}

		auto set_rb_axis = [](const bool is_detected, const glm::fmat4x4& mat_frm2ws, int& obj_id)
		{
			if (is_detected)
			{
				Axis_Gen(mat_frm2ws, 0.07, obj_id);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_id, default_obj_state);
			}
			else if (obj_id != 0)
			{
				vzm::ObjStates ostate = default_obj_state;
				ostate.is_visible = false;
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, obj_id, ostate);
			}
		};
		set_rb_axis(is_rsrb_detected, mat_clf2ws, g_info.otrk_data.rs_lf_axis_id);
		set_rb_axis(is_probe_detected, mat_probe2ws, g_info.otrk_data.probe_lf_axis_id);
	}

	void RecordInfo(const int key_pressed, const void* color_data)
	{
		record_trk_info.push_back(g_info.otrk_data.trk_info);
		char* img_data = new char[g_info.rs_w * g_info.rs_h * 3];
		memcpy(img_data, color_data, sizeof(char) * 3 * g_info.rs_w * g_info.rs_h);
		record_rsimg.push_back(img_data);
		record_key.push_back(key_pressed);
	}

	void SetTcCalibMkPoints(bool is_visible)
	{
		auto marker_color = [](int idx, int w)
		{
			return glm::fvec3((idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1), 1);
		};
		if (is_visible)
		{
			vector<glm::fvec4> sphers_xyzr;
			vector<glm::fvec3> sphers_rgb;
			for (int i = 0; i < g_info.otrk_data.calib_3d_pts.size(); i++)
			{
				Point3f pt = g_info.otrk_data.calib_3d_pts[i];
				sphers_xyzr.push_back(glm::fvec4(pt.x, pt.y, pt.z, 0.007));
				sphers_rgb.push_back(marker_color(i, (int)g_info.otrk_data.calib_3d_pts.size() / 2));
			}
			if (sphers_xyzr.size() > 0)
			{
				vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0],
					g_info.otrk_data.calib_3d_pts.size(), g_info.otrk_data.cb_spheres_id);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.cb_spheres_id, default_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.cb_spheres_id, default_obj_state);
			}
			else
			{
				vzm::ObjStates cstate = default_obj_state;
				cstate.is_visible = false;
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
			}
		}
		else
		{
			vzm::ObjStates cstate = default_obj_state;
			cstate.is_visible = false;
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.cb_spheres_id, cstate);
		}
	}

	void SetMkSpheres(bool is_visible, bool is_pickable)
	{
		auto marker_color = [](int idx, int w)
		{
			return glm::fvec3(1, (idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1));
		};

		auto register_mks = [](const glm::fvec3* pos_list, const int num_mks, const float r, int& mks_id)
		{
			vector<glm::fvec4> sphers_xyzr;
			vector<glm::fvec3> sphers_rgb;
			for (int i = 0; i < num_mks; i++)
			{
				glm::fvec3 pt = pos_list[i];
				sphers_xyzr.push_back(glm::fvec4(pt.x, pt.y, pt.z, r));
				sphers_rgb.push_back(glm::fvec3(1, 1, 0));
			}

			if (num_mks > 0)
				vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], num_mks, mks_id);
			else
			{
				vzm::DeleteObject(mks_id);
				mks_id = 0;
			}
		};

		is_pickable |= g_info.touch_mode == RsTouchMode::Calib_STG;

		if (is_visible && !is_pickable)
		{
			register_mks((glm::fvec3*)&g_info.otrk_data.trk_info.mk_xyz_list[0], g_info.otrk_data.trk_info.mk_xyz_list.size() / 3, 0.005, g_info.otrk_data.mks_spheres_id);
			if (g_info.otrk_data.mks_spheres_id != 0)
			{
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.mks_spheres_id, default_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.mks_spheres_id, default_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.otrk_data.mks_spheres_id, default_obj_state);
			}
		}
		else
		{
			vzm::ObjStates cstate = default_obj_state;
			cstate.is_visible = false;
			{
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.mks_spheres_id, cstate);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.otrk_data.mks_spheres_id, cstate);
				vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.otrk_data.mks_spheres_id, cstate);
			}
		}

		if (is_pickable)
		{
			int num_mks = g_info.otrk_data.trk_info.mk_xyz_list.size() / 3;
			g_info.vzmobjid2pos.clear();
			for (int i = 0; i < (int)g_info.otrk_data.mk_pickable_sphere_ids.size(); i++)
				vzm::DeleteObject(g_info.otrk_data.mk_pickable_sphere_ids[i]);
			g_info.otrk_data.mk_pickable_sphere_ids.clear();

			for (int i = 0; i < num_mks; i++)
			{
				g_info.otrk_data.mk_pickable_sphere_ids.push_back(0);
				int& pickable_mk_id = g_info.otrk_data.mk_pickable_sphere_ids[i];
				glm::fvec3 pt = g_info.otrk_data.trk_info.GetMkPos(i);
				vzm::GenerateSpheresObject(__FP glm::fvec4(pt.x, pt.y, pt.z, 0.015), __FP marker_color(i, 7), 1, pickable_mk_id);
				g_info.vzmobjid2pos[pickable_mk_id] = pt;
				vzm::ValidatePickTarget(pickable_mk_id);
				vzm::ObjStates cstate = default_obj_state;
				if (g_info.touch_mode == RsTouchMode::Calib_STG)
					vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, pickable_mk_id, cstate);
				else
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, pickable_mk_id, cstate);
			}
		}
		else
		{
			for (int i = 0; i < (int)g_info.otrk_data.mk_pickable_sphere_ids.size(); i++)
				vzm::DeleteObject(g_info.otrk_data.mk_pickable_sphere_ids[i]);
			g_info.otrk_data.mk_pickable_sphere_ids.clear();
			g_info.vzmobjid2pos.clear();
		}
	}

	void GetVarInfo(void* ginfo)
	{
		*(GlobalInfo*)ginfo = g_info;
	}

	void SetVarInfo(const void* ginfo)
	{
		g_info = *(GlobalInfo*)ginfo;
	}

	void TryCalibrationTC(Mat& imgColor)
	{
		auto marker_color = [](int idx, int w)
		{
			return glm::fvec3((idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1), 1);
		};
		if (g_info.touch_mode == RsTouchMode::Calib_TC && is_rsrb_detected && g_info.otrk_data.calib_3d_pts.size() > 0)
		{
			// calibration routine
			Mat viewGray;
			cvtColor(imgColor, viewGray, COLOR_BGR2GRAY);

			std::vector<__MarkerDetInfo> list_det_armks;
			ar_marker.track_markers(list_det_armks, viewGray.data, viewGray.cols, viewGray.rows, mk_ids);

			for (int i = 0; i < (int)list_det_armks.size(); i++)
			{
				__MarkerDetInfo& armk = list_det_armks[i];

				int id = armk.id;
				glm::fvec3 _rgb = marker_color(id - 1, g_info.otrk_data.calib_3d_pts.size() / 2);

				for (int j = 0; j < 4; j++)
					circle(imgColor, Point(armk.corners2d[2 * j + 0], armk.corners2d[2 * j + 1]), 1, CV_RGB(_rgb.r * 255, _rgb.g * 255, _rgb.b * 255), 2);
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

				if (glm::length(diff) > 0.05 && point2d.size() > 0)
				{
					prev_mat_clf2ws = mat_clf2ws;
					float pnp_err = -1.f;
					int calib_samples = 0;
					bool is_success = CalibrteCamLocalFrame(*(vector<glm::fvec2>*)&point2d, *(vector<glm::fvec3>*)&point3d, mat_ws2clf,
						rs_settings::rgb_intrinsics.fx, rs_settings::rgb_intrinsics.fy, rs_settings::rgb_intrinsics.ppx, rs_settings::rgb_intrinsics.ppy,
						mat_rscs2clf, &pnp_err, &calib_samples, g_info.otrk_data.tc_calib_pt_pairs);
					if (is_success)
					{
						g_info.is_calib_rs_cam = true;
						// 
						ofstream outfile(g_info.rs_calib);
						if (outfile.is_open())
						{
							outfile.clear();

							outfile << to_string(point2d.size()) << endl;
							for (int i = 0; i < (int)point2d.size(); i++)
							{
								Point2d p2d = point2d[i];
								Point3d p3d = point3d[i];
								string line = to_string(p2d.x) + " " + to_string(p2d.y) + " " + to_string(p3d.x) + " " + to_string(p3d.y) + " " + to_string(p3d.z);
								outfile << line << endl;
							}

							float* d = glm::value_ptr(mat_rscs2clf);
							for (int i = 0; i < 16; i++)
							{
								string line = to_string(d[i]);
								outfile << line << endl;
							}
						}
						outfile.close();
					}

					for (int i = 0; i < g_info.otrk_data.calib_trial_rs_cam_frame_ids.size(); i++)
					{
						vzm::ObjStates cstate;
						vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.otrk_data.calib_trial_rs_cam_frame_ids[i], cstate);
						cstate.is_visible = true;
						vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.calib_trial_rs_cam_frame_ids[i], cstate);
					}

					int calib_frame_id = 0;
					Axis_Gen(mat_clf2ws, 0.05f, calib_frame_id);
					vzm::ObjStates cstate = default_obj_state;
					cstate.color[3] = 0.3f;
					vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, calib_frame_id, cstate);
					g_info.otrk_data.calib_trial_rs_cam_frame_ids.push_back(calib_frame_id);
				}
			}
		}

		if (g_info.is_calib_rs_cam)
		{
			glm::fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;

			//rs_cam_tris_id, rs_cam_lines_id, rs_cam_txt_id
			if (is_rsrb_detected)
				Update_CamModel(g_info.ws_scene_id, mat_rscs2ws, "RS CAM 0", 2);

			vzm::CameraParameters _rs_cam_params;
			vzm::GetCameraParameters(g_info.rs_scene_id, _rs_cam_params, rs_cam_id);
			ComputeCameraStates(mat_rscs2clf, mat_clf2ws, _rs_cam_params);
			vzm::SetCameraParameters(g_info.rs_scene_id, _rs_cam_params, rs_cam_id);

			vzm::SceneEnvParameters scn_env_params;
			vzm::GetSceneEnvParameters(g_info.ws_scene_id, scn_env_params);
			scn_env_params.is_on_camera = false;
			__cv3__ scn_env_params.pos_light = __cv3__ _rs_cam_params.pos;
			__cv3__ scn_env_params.dir_light = __cv3__ _rs_cam_params.view;
			vzm::SetSceneEnvParameters(g_info.ws_scene_id, scn_env_params);
		}
	}

	void TryCalibrationSTG()
	{
		static int mk_stg_calib_sphere_id = 0;
		static int clf_mk_stg_calib_spheres_id = 0;
		static int last_calib_pair = 0;

#ifdef STG_LINE_CALIB
		static Point2d pos_calib_lines[4] = { Point2d(100, 100), Point2d(400, 400), Point2d(400, 100), Point2d(100, 400) };
#endif
		if (g_info.touch_mode == RsTouchMode::Calib_STG)
		{
			int stg_calib_mk_idx;
			bool exist_mk_cid = g_info.otrk_data.trk_info.CheckExistCID(g_info.otrk_data.stg_calib_mk_cid, &stg_calib_mk_idx);
			if (exist_mk_cid)
			{
				glm::fvec3 pos_stg_calib_mk = g_info.otrk_data.trk_info.GetMkPos(stg_calib_mk_idx);
				vzm::GenerateSpheresObject(__FP glm::fvec4(pos_stg_calib_mk, 0.02), __FP glm::fvec3(1), 1, mk_stg_calib_sphere_id);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, mk_stg_calib_sphere_id, default_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, mk_stg_calib_sphere_id, default_obj_state);
			}
			else
			{
				vzm::ObjStates cstate;
				vzm::GetSceneObjectState(g_info.ws_scene_id, mk_stg_calib_sphere_id, cstate);
				cstate.is_visible = false;
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, mk_stg_calib_sphere_id, cstate);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, mk_stg_calib_sphere_id, cstate);
			}

			int num_stg_calib_pairs = (int)g_info.otrk_data.stg_calib_pt_pairs.size();

			vector<Point2f> point2d;
			vector<Point3f> point3d;
			vector<glm::fvec4> ws_mk_spheres_xyzr;
			vector<glm::fvec3> ws_mk_spheres_rgb;
			for (int i = 0; i < num_stg_calib_pairs; i++)
			{
				pair<Point2f, Point3f>& pr = g_info.otrk_data.stg_calib_pt_pairs[i];
				point2d.push_back(get<0>(pr));
				Point3f pos_pt = get<1>(pr);
				point3d.push_back(pos_pt);

				glm::fvec3 pos_mk_ws = tr_pt(mat_clf2ws, __cv3__ &pos_pt);
				ws_mk_spheres_xyzr.push_back(glm::fvec4(pos_mk_ws, 0.007));
				ws_mk_spheres_rgb.push_back(glm::fvec3(0.2, 0.8, 1));
			}

			if (num_stg_calib_pairs > 0)
			{
				vzm::GenerateSpheresObject(__FP ws_mk_spheres_xyzr[0], __FP ws_mk_spheres_rgb[0], num_stg_calib_pairs, clf_mk_stg_calib_spheres_id);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, clf_mk_stg_calib_spheres_id, default_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, clf_mk_stg_calib_spheres_id, default_obj_state);
			}
			else
			{
				vzm::DeleteObject(clf_mk_stg_calib_spheres_id);
			}

			if (num_stg_calib_pairs >= 12 && last_calib_pair != num_stg_calib_pairs)
			{
				cout << "compute STG calibration" << endl;
				last_calib_pair = num_stg_calib_pairs;
				vzm::CameraParameters cam_state_calbirated;

				// calculate intrinsics and extrinsics
				// crbs means Camera RigidBody Space
				{
					glm::fvec3 pos_crbs, view_crbs, up_crbs;
					glm::fmat4x4 mat_clf2clf;
					helpers::ComputeArCameraCalibrateInfo(__FP mat_clf2clf, __FP point3d[0], __FP point2d[0], num_stg_calib_pairs,
						__FP mat_stgcs2clf, &cam_state_calbirated);

					float* pv = glm::value_ptr(mat_stgcs2clf);
					for(int i =0; i < 16; i++)
					std::cout << i << " : " << pv[i] << std::endl;

					// why divide by 4?!
					//cam_state_calbirated.fx /= 4.f;
					//cam_state_calbirated.fy /= 4.f;
					//cam_state_calbirated.cx /= 4.f;
					//cam_state_calbirated.cy /= 4.f;
					//cam_state_calbirated.sc /= 4.f;
				}

				cam_state_calbirated.w = g_info.stg_w;
				cam_state_calbirated.h = g_info.stg_h;
				cam_state_calbirated.np = 0.1f;
				cam_state_calbirated.fp = 20.0f;
				cam_state_calbirated.projection_mode = 3;

				// 
				__cv3__ cam_state_calbirated.pos = tr_pt(mat_clf2ws, __cv3__ cam_state_calbirated.pos);
				__cv3__ cam_state_calbirated.up = glm::normalize(tr_vec(mat_clf2ws, __cv3__ cam_state_calbirated.up));
				__cv3__ cam_state_calbirated.view = glm::normalize(tr_vec(mat_clf2ws, __cv3__ cam_state_calbirated.view));

				vzm::SetCameraParameters(g_info.stg_scene_id, cam_state_calbirated, stg_cam_id);
				g_info.is_calib_stg_cam = true;
				// TO DO //
				//bool is_success = CalibrteCamLocalFrame(*(vector<glm::fvec2>*)&point2d, *(vector<glm::fvec3>*)&point3d, mat_ws2clf,
				//	rgb_intrinsics.fx, rgb_intrinsics.fy, rgb_intrinsics.ppx, rgb_intrinsics.ppy,
				//	mat_rscs2clf, &pnp_err, &calib_samples, 3d, 2d);
			}

		}
		else
		{
			last_calib_pair = 0;
			vzm::ObjStates cstate = default_obj_state;
			cstate.is_visible = false;
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, mk_stg_calib_sphere_id, cstate);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, mk_stg_calib_sphere_id, cstate);

			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, clf_mk_stg_calib_spheres_id, cstate);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, clf_mk_stg_calib_spheres_id, cstate);
		}

		if (g_info.is_calib_stg_cam)
		{
			glm::fmat4x4 mat_stgcs2ws = mat_clf2ws * mat_stgcs2clf;

			//rs_cam_tris_id, rs_cam_lines_id, rs_cam_txt_id
			if (is_rsrb_detected)
				Update_CamModel(g_info.ws_scene_id, mat_stgcs2ws, "STG CAM", 3);

			vzm::CameraParameters _stg_cam_params;
			vzm::GetCameraParameters(g_info.stg_scene_id, _stg_cam_params, stg_cam_id);
			ComputeCameraStates(mat_stgcs2clf, mat_clf2ws, _stg_cam_params);
			vzm::SetCameraParameters(g_info.stg_scene_id, _stg_cam_params, stg_cam_id);

			vzm::CameraParameters rs_cam_params;
			vzm::GetCameraParameters(g_info.rs_scene_id, rs_cam_params, rs_cam_id);
		}
	}

	void SetCalibFrames(bool is_visible)
	{
		vzm::ObjStates cstate = default_obj_state;
		cstate.color[3] = 0.3f;
		cstate.is_visible = is_visible;
		for (int i = 0; i < g_info.otrk_data.calib_trial_rs_cam_frame_ids.size(); i++)
		{
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.otrk_data.calib_trial_rs_cam_frame_ids[i], cstate);
		}
	}

	void SetDepthMapPC(const bool is_visible, rs2::depth_frame& depth_frame, rs2::video_frame& color_frame)
	{
		if (is_visible && depth_frame)
		{
			//rs2::depth_frame depth_frame = depth_frame;// .get_depth_frame();

			vzm::ObjStates obj_state_pts = default_obj_state;
			obj_state_pts.color[3] = 1.f;
			obj_state_pts.emission = 0.3f;
			obj_state_pts.diffusion = 1.f;
			obj_state_pts.point_thickness = 2.f;
			// Generate the pointcloud and texture mappings
			rs_settings::pc.map_to(color_frame); // before pc.calculate, which generates texture_coordinates
			rs_settings::points = rs_settings::pc.calculate(depth_frame);
			if (rs_settings::points)
			{
				auto vertices = rs_settings::points.get_vertices();              // get vertices
				auto tex_coords = rs_settings::points.get_texture_coordinates(); // and texture coordinates

				vzm::CameraParameters _cam_params;
				vzm::GetCameraParameters(g_info.ws_scene_id, _cam_params, ov_cam_id);

				//vector<glm::fvec3> vtx;// (points.size());
				//memcpy(&vtx[0], &vertices[0], sizeof(glm::fvec3) * points.size());
				glm::fmat4x4 mat_r = glm::rotate(-glm::pi<float>(), glm::fvec3(1, 0, 0));
				glm::fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;
				glm::fmat4x4 mat_os2ws;


				{
					glm::fmat4x4 mat_rs2ws = mat_rscs2ws * mat_r;
					const float *rv = rs_settings::rgb_extrinsics.rotation;
					glm::fmat4x4 mat_rt(rv[0], rv[1], rv[2], 0, rv[3], rv[4], rv[5], 0, rv[6], rv[7], rv[8], 0,
						rs_settings::rgb_extrinsics.translation[0], rs_settings::rgb_extrinsics.translation[1], rs_settings::rgb_extrinsics.translation[2], 1); // ignore 4th row 
					//*(glm::fmat4x4*) obj_state_pts.os2ws = mat_rs2ws * mat_rt;
					mat_os2ws = mat_rs2ws * mat_rt; // depth to rgb (external)
				}
				glm::fvec3* normalmap = NULL;
				const int _w = depth_frame.as<rs2::video_frame>().get_width();
				const int _h = depth_frame.as<rs2::video_frame>().get_height();
				{
					// compute face normal
					normalmap = new glm::fvec3[_w * _h];
					auto depth_color = depth_frame.apply_filter(rs_settings::color_map);
					Mat image_depth(Size(_w, _h), CV_8UC3, (void*)depth_color.get_data(), Mat::AUTO_STEP);
					imshow("test depth", image_depth);

					// 
					glm::fmat4x4 mat_irss2os = mat_r * rs_settings::mat_irss2ircs;
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

				}
				vector<glm::fvec2> tex(rs_settings::points.size());
				memcpy(&tex[0], &tex_coords[0], sizeof(glm::fvec2) * rs_settings::points.size());
				vector<glm::fvec3> color_pts(rs_settings::points.size());
				vector<glm::fvec3> pos_pts(rs_settings::points.size());
				vector<glm::fvec3> nrl_pts(rs_settings::points.size());
				for (int i = 0; i < (int)rs_settings::points.size(); i++)
				{
					float tx = tex[i].x * g_info.rs_w;
					float ty = tex[i].y * g_info.rs_h;
					int _tx = (int)tx;
					int _ty = (int)ty;
					if (_tx < 0 || _ty < 0 || _tx >= g_info.rs_w || _ty >= g_info.rs_h)
						continue;
					glm::u8vec3* _data = (glm::u8vec3*)color_frame.get_data();
					glm::u8vec3 _color0 = _data[_tx + _ty * g_info.rs_w];
					//glm::u8vec3 _color1 = _data[_tx + _ty * g_info.rs_w];
					//glm::u8vec3 _color2 = _data[_tx + _ty * g_info.rs_w];
					//glm::u8vec3 _color3 = _data[_tx + _ty * g_info.rs_w];
					color_pts[i] = glm::fvec3(_color0.x / 255.f, _color0.y / 255.f, 1.f);// _color0.z / 255.f);
					pos_pts[i] = tr_pt(mat_os2ws, *(glm::fvec3*)&vertices[i]);
					if (normalmap) nrl_pts[i] = normalmap[i % _w + (i / _w) * _w];
				}
				if (normalmap) delete[] normalmap;
				//vzm::GeneratePointCloudObject(__FP pos_pts[0], NULL, __FP color_pts[0], (int)points.size(), g_info.rs_pc_id);
				vzm::GeneratePointCloudObject(__FP pos_pts[0], __FP nrl_pts[0], NULL, (int)rs_settings::points.size(), g_info.rs_pc_id);
				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.rs_pc_id, obj_state_pts);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.rs_pc_id, obj_state_pts);
				vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.rs_pc_id, obj_state_pts);
				bool foremost_surf_rendering = true;
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
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.rs_pc_id, obj_state_pts);
		}
	}

	void SetTargetModelAssets(const std::string& name)
	{
		if (g_info.model_ws_pick_spheres_id != 0)
		{
			vzm::ObjStates cstate;
			vzm::GetSceneObjectState(g_info.rs_scene_id, g_info.model_ws_pick_spheres_id, cstate);
			cstate.is_visible = g_info.touch_mode == RsTouchMode::Align;
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.model_ws_pick_spheres_id, cstate);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.model_ws_pick_spheres_id, cstate);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.model_ws_pick_spheres_id, cstate);
		}

		glm::fmat4x4 mat_matchmodelfrm2ws;
		bool model_match_rb = g_info.otrk_data.trk_info.GetLFrmInfo(name, mat_matchmodelfrm2ws); 
		g_info.mat_ws2matchmodelfrm = glm::inverse(mat_matchmodelfrm2ws);	// 변경 위치

		if (model_match_rb && g_info.is_modelaligned)
		{
			//g_info.mat_ws2matchmodelfrm = glm::inverse(mat_matchmodelfrm2ws);	// 기존 위치

			vzm::ObjStates model_ws_obj_state;
			vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);

			__cm4__ model_ws_obj_state.os2ws = mat_matchmodelfrm2ws * g_info.mat_os2matchmodefrm;
			if (scenario == 1)
			{
				vzm::ObjStates volume_ws_obj_state;
				vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.model_volume_id, volume_ws_obj_state);
				__cm4__ volume_ws_obj_state.os2ws = mat_matchmodelfrm2ws * g_info.mat_os2matchmodefrm;
				volume_ws_obj_state.is_visible = true;

				vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.model_volume_id, volume_ws_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.model_volume_id, volume_ws_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.model_volume_id, volume_ws_obj_state);

				model_ws_obj_state.color[3] = 0.05f;
				model_ws_obj_state.point_thickness = 15;
			}
			vzm::ReplaceOrAddSceneObject(g_info.ws_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.rs_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);
			vzm::ReplaceOrAddSceneObject(g_info.stg_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);
		}
	}

	void SetSectionalImageAssets(const bool show_sectional_views, const float* _pos_tip, const float* _pos_end)
	{
		// ws
		glm::fvec3 pos_tip = __cv3__ _pos_tip;
		glm::fvec3 pos_end = __cv3__ _pos_end;

		// after calling SetTargetModelAssets
		_show_sectional_views = show_sectional_views;
		if (show_sectional_views)
		{
			vzm::ObjStates model_ws_obj_state;
			vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);
			if (g_info.model_volume_id == 0)
			{
				vzm::ReplaceOrAddSceneObject(g_info.csection_scene_id, g_info.model_ws_obj_id, model_ws_obj_state);
			}
			else
			{
				vzm::ObjStates volume_ws_obj_state;
				vzm::GetSceneObjectState(g_info.ws_scene_id, g_info.model_volume_id, volume_ws_obj_state);
				vzm::ReplaceOrAddSceneObject(g_info.csection_scene_id, g_info.model_volume_id, volume_ws_obj_state);
			}

			vzm::CameraParameters csection_cam_params_model;
			csection_cam_params_model.np = 0.0f;
			csection_cam_params_model.fp = 10.0f;
			csection_cam_params_model.projection_mode = 4;
			csection_cam_params_model.ip_w = 0.1;
			csection_cam_params_model.ip_h = 0.1;
			csection_cam_params_model.w = 180;
			csection_cam_params_model.h = 180;

			__cv3__ csection_cam_params_model.pos = pos_tip; // g_info.pos_probe_pin;
			glm::fvec3 cs_up = glm::normalize(pos_end - pos_tip);// tr_vec(mat_section_probe2ws, glm::fvec3(0, 0, -1));
			__cv3__ csection_cam_params_model.up = cs_up;

			glm::fvec3 cs_view = glm::fvec3(0, 0, 1);
			glm::fvec3 cs_right = glm::cross(cs_view, cs_up);
			cs_view = glm::normalize(glm::cross(cs_up, cs_right));
			__cv3__ csection_cam_params_model.view = cs_view;
			vzm::SetCameraParameters(g_info.csection_scene_id, csection_cam_params_model, 0);

			cs_view = glm::fvec3(1, 0, 0);
			cs_right = glm::cross(cs_view, cs_up);
			cs_view = glm::normalize(glm::cross(cs_up, cs_right));
			__cv3__ csection_cam_params_model.view = cs_view;
			vzm::SetCameraParameters(g_info.csection_scene_id, csection_cam_params_model, 1);
		}
	}

	void RenderAndShowWindows(bool show_times, Mat& img_rs, bool skip_show_rs_window)
	{
		auto DisplayTimes = [&show_times](const LARGE_INTEGER lIntCntStart, const string& _test)
		{
			if (!show_times) return;
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

		if (!g_info.skip_call_render)
		{
#define ENABLE_STG
#define SHOW_WS_VIEW
#define SHOW_RS_VIEW
#define SHOW_MS_VIEW
#define SHOW_SECTION_VIEW
#define SHOW_STG_VIEW
#ifdef SHOW_WS_VIEW
			LARGE_INTEGER frq_render_ws = GetPerformanceFreq();

			string tc_calib_info = "# of current 3D pick positions : " + to_string(g_info.otrk_data.calib_3d_pts.size());
			Show_Window(g_info.window_name_ws_view, g_info.ws_scene_id, ov_cam_id, &tc_calib_info);
			DisplayTimes(frq_render_ws, "ws render : ");
#endif

#ifdef SHOW_RS_VIEW
			if (g_info.is_calib_rs_cam)
			{
				LARGE_INTEGER frq_render_rs = GetPerformanceFreq();
				vzm::RenderScene(g_info.rs_scene_id, rs_cam_id);
				DisplayTimes(frq_render_rs, "rs render : ");
				unsigned char* ptr_rgba;
				float* ptr_zdepth;
				int rs_w, rs_h;
				LARGE_INTEGER frq_render_cb = GetPerformanceFreq();
				if (vzm::GetRenderBufferPtrs(g_info.rs_scene_id, &ptr_rgba, &ptr_zdepth, &rs_w, &rs_h, rs_cam_id))
					copy_back_ui_buffer(img_rs.data, ptr_rgba, rs_w, rs_h, false);


				if (_show_sectional_views)
				{
					vzm::RenderScene(g_info.csection_scene_id, 0);
					vzm::RenderScene(g_info.csection_scene_id, 1);

					for (int i = 0; i < 2; i++)
					{
						unsigned char* cs_ptr_rgba;
						float* cs_ptr_zdepth;
						int cs_w, cs_h;
						vzm::GetRenderBufferPtrs(g_info.csection_scene_id, &cs_ptr_rgba, &cs_ptr_zdepth, &cs_w, &cs_h, i);
						cv::Mat cs_cvmat(cs_h, cs_w, CV_8UC4, cs_ptr_rgba);
						cv::line(cs_cvmat, cv::Point(cs_w / 2, cs_h / 2), cv::Point(cs_w / 2, 0), cv::Scalar(255, 255, 0), 2, 2);
						cv::circle(cs_cvmat, cv::Point(cs_w / 2, cs_h / 2), 2, cv::Scalar(255, 0, 0), 2);

						// to do //
						copy_back_ui_buffer_local(img_rs.data, rs_w, rs_h, cs_ptr_rgba, cs_w, cs_h, rs_w - cs_w, rs_h - cs_h - cs_h * i, false);
					}
				}

				DisplayTimes(frq_render_rs, "rs copy-back : ");
			}

			// draw buttons
			Draw_TouchButtons(img_rs, g_info.rs_buttons, g_info.touch_mode);

			if (g_info.is_calib_rs_cam && !is_rsrb_detected)
				cv::putText(img_rs, "RS Cam is out of tracking volume !!", cv::Point(0, 150), cv::FONT_HERSHEY_DUPLEX, 2.0, CV_RGB(255, 0, 0), 3, LineTypes::LINE_AA);

			if(!skip_show_rs_window)
				imshow(g_info.window_name_rs_view, img_rs);

#ifdef __MIRRORS
			{
				cv::Mat img_rs_mirror(g_info.rs_h, g_info.rs_w, CV_8UC3, img_rs.data);
				imshow("rs mirror", img_rs_mirror);
			}
#endif
#endif

#if defined(ENABLE_STG) && defined(SHOW_STG_VIEW)
			auto Draw_STG_Calib_Point = [](Mat& img)
			{
				if (g_info.touch_mode == RsTouchMode::Calib_STG)
				{
					const int w = g_info.stg_w;
					const int h = g_info.stg_h;
					static Point2f pos_2d_rs[15] = {
						Point2f(w / 5.f, h / 4.f) , Point2f(w / 5.f * 2.f, h / 4.f) , Point2f(w / 5.f * 3.f, h / 4.f) , Point2f(w / 5.f * 4.f, h / 4.f),
						Point2f(w / 8.f, h / 4.f * 2.f) , Point2f(w / 8.f * 2.f, h / 4.f * 2.f) , Point2f(w / 8.f * 3.f, h / 4.f * 2.f) , Point2f(w / 8.f * 4.f, h / 4.f * 2.f),
						Point2f(w / 8.f * 5.f, h / 4.f * 2.f) , Point2f(w / 8.f * 6.f, h / 4.f * 2.f) , Point2f(w / 8.f * 7.f, h / 4.f * 2.f),
						Point2f(w / 5.f, h / 4.f * 3.f) , Point2f(w / 5.f * 2.f, h / 4.f * 3.f) , Point2f(w / 5.f * 3.f, h / 4.f * 3.f) , Point2f(w / 5.f * 4.f, h / 4.f * 3.f) };


					if (g_info.otrk_data.stg_calib_pt_pairs.size() < 15)
						cv::drawMarker(img, pos_2d_rs[g_info.otrk_data.stg_calib_pt_pairs.size()], Scalar(255, 100, 100), MARKER_CROSS, 30, 3);
					else
						for (int i = 0; i < g_info.otrk_data.stg_calib_pt_pairs.size(); i++)
						{
							pair<Point2f, Point3f>& pair_pts = g_info.otrk_data.stg_calib_pt_pairs[i];
							cv::drawMarker(img, get<0>(pair_pts), Scalar(255, 255, 100), MARKER_STAR, 30, 3);
						}

#ifdef STG_LINE_CALIB
					cv::line(image_stg, pos_calib_lines[0], pos_calib_lines[1], Scalar(255, 255, 0), 2);
					cv::line(image_stg, pos_calib_lines[2], pos_calib_lines[3], Scalar(255, 255, 0), 2);
#endif
				}
			};
			if (g_info.is_calib_stg_cam)
			{
				LARGE_INTEGER frq_render_stg = GetPerformanceFreq();
				vzm::RenderScene(g_info.stg_scene_id, stg_cam_id);
				unsigned char* ptr_rgba;
				float* ptr_zdepth;
				int _stg_w, _stg_h;
				if (vzm::GetRenderBufferPtrs(g_info.stg_scene_id, &ptr_rgba, &ptr_zdepth, &_stg_w, &_stg_h, stg_cam_id))
				{

					Mat image_stg(Size(_stg_w, _stg_h), CV_8UC4, (void*)ptr_rgba, Mat::AUTO_STEP);
					cv::drawMarker(image_stg, Point(_stg_w / 2, _stg_h / 2), Scalar(255, 255, 255), MARKER_CROSS, 30, 3);
					cv::rectangle(image_stg, Point(0, 0), Point(g_info.stg_w - 10, g_info.stg_h - 5), Scalar(255, 255, 255), 3);
					Draw_STG_Calib_Point(image_stg);

					imshow(g_info.window_name_stg_view, image_stg);
					
#ifdef __MIRRORS
					cv::Mat img_stg_mirror(Size(_stg_w, _stg_h), CV_8UC4, image_stg.data);
					imshow("stg mirror", img_stg_mirror);
#endif
				}

				DisplayTimes(frq_render_stg, "stg render : ");
			}
			else
			{
				static Mat image_stg(Size(g_info.stg_w, g_info.stg_h), CV_8UC4, Mat::AUTO_STEP);
				image_stg = cv::Mat::zeros(image_stg.size(), image_stg.type());
				cv::drawMarker(image_stg, Point(g_info.stg_w / 2, g_info.stg_h / 2), Scalar(100, 100, 255), MARKER_CROSS, 30, 3);
				cv::rectangle(image_stg, Point(0, 0), Point(g_info.stg_w - 10, g_info.stg_h - 5), Scalar(255, 255, 255), 3);
				Draw_STG_Calib_Point(image_stg);
				imshow(g_info.window_name_stg_view, image_stg);

#ifdef __MIRRORS
				cv::Mat img_stg_mirror(Size(g_info.stg_w, g_info.stg_h), CV_8UC4, image_stg.data);
				imshow("stg mirror", img_stg_mirror);
#endif
			}
#endif
		}
	}

	void DeinitializeVarSettings()
	{
		clear_record_info();
	}
}