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

#define __NUMMARKERS 15
#include "../optitrk/optitrack.h"

using namespace std;
using namespace cv;

#include "../kar_helpers.hpp"

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

	// configure file path
const string window_name_rs_view = "RealSense VIEW";
const string window_name_ws_view = "World VIEW";
const string window_name_ms_view = "Model VIEW";

const string optrack_calib = "D:\\Document\\OptiTrack\\my_test_200810_1.cal";
const string optrack_env = "D:\\Document\\OptiTrack\\my_test_200810_1.motive";
const string cb_positions = "E:\\project_srcs\\kar\\prototype_ver1\\cb_points.txt";
const string sst_positions = "E:\\project_srcs\\kar\\prototype_ver1\\ss_pin_pts.txt";
//const string model_path = "D:\\Data\\K-AR_Data\\demo.obj";
const string model_path = "D:\\Data\\K-AR_Data\\brain\\1\\skin_c_output.obj";

ENUM(RsMouseMode, NONE, ADD_CALIB_POINTS, GATHERING_POINTS, PIN_ORIENTATION)
RsMouseMode rs_ms_mode = NONE;
bool skip_main_thread = false;

void CallBackFunc_WorldMouse(int event, int x, int y, int flags, void* userdata)
{
	glm::ivec2 scene_cam = *(glm::ivec2*)userdata;
	vzm::CameraParameters cam_params;
	vzm::GetCameraParameters(scene_cam.x, cam_params, scene_cam.y);

	// https://docs.opencv.org/3.4/d7/dfc/group__highgui.html
	static helpers::arcball aball_ov;
	if (event == EVENT_LBUTTONDOWN || event == EVENT_RBUTTONDOWN)
	{
		aball_ov.intializer((float*)&glm::fvec3(), 2.0f);

		helpers::cam_pose arc_cam_pose;
		glm::fvec3 pos = __cv3__ arc_cam_pose.pos = __cv3__ cam_params.pos;
		__cv3__ arc_cam_pose.up = __cv3__ cam_params.up;
		__cv3__ arc_cam_pose.view = __cv3__ cam_params.view;
		aball_ov.start((int*)&glm::ivec2(x, y), (float*)&glm::fvec2(cam_params.w, cam_params.h), arc_cam_pose);
	}
	else if (event == EVENT_MBUTTONDOWN)
	{
	}
	else if (event == EVENT_MOUSEWHEEL)
	{
		if (getMouseWheelDelta(flags) > 0)
			__cv3__ cam_params.pos += 0.05f * (__cv3__ cam_params.view);
		else
			__cv3__ cam_params.pos -= 0.05f * (__cv3__ cam_params.view);
		vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
	}
	else if (event == EVENT_MOUSEMOVE)
	{
		if (flags & EVENT_FLAG_LBUTTON)
		{
			helpers::cam_pose arc_cam_pose;
			aball_ov.pan_move((int*)&glm::ivec2(x, y), arc_cam_pose);
			__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
			__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
			__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
			vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
		}
		else if (flags & EVENT_FLAG_RBUTTON)
		{
			helpers::cam_pose arc_cam_pose;
			aball_ov.move((int*)&glm::ivec2(x, y), arc_cam_pose);
			__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
			__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
			__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
			vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
		}
	}
}

struct OpttrkData
{
	track_info trk_info; // available when USE_OPTITRACK
	vzm::ObjStates obj_state;
	int cb_spheres_id;
	vector<Point3f> calib_3d_pts;

	OpttrkData()
	{
		cb_spheres_id = 0;
		obj_state.emission = 0.4f;
		obj_state.diffusion = 0.6f;
		obj_state.specular = 0.2f;
		obj_state.sp_pow = 30.f;
		__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
		__cm4__ obj_state.os2ws = glm::fmat4x4();
	}
};

OpttrkData g_otrk_data;
bool do_initialize_trk_points = false;
float pnp_err = -1.f;
int num_calib = 0;
int calib_samples = 0;
int rs_pc_id = 0;
int rs_lf_axis = 0, probe_lf_axis = 0, sstool_lf_axis = 0;
bool show_calib_frames = true;
bool show_pc = false;
bool calib_toggle = false;
int mesh_obj_id = 0;
glm::fmat4x4 mat_rscs2clf;
bool is_calib_cam = false;
glm::fvec3 pos_probe_pin;
int sample_rs_pts_ids[3] = {};
glm::fvec3 sample_pts_ws[3];
struct SS_Tool_Guide_Pts
{
	int ss_tool_guide_points_id;
	vector<glm::fvec3> pos_centers_tfrm;
	SS_Tool_Guide_Pts() { ss_tool_guide_points_id = 0; }
};
SS_Tool_Guide_Pts ss_tool_info;

// scene definition
const int ws_scene_id = 1; // arbitrary integer
const int rs_scene_id = 2; // arbitrary integer
const int model_scene_id = 3; // arbitrary integer
const int csection_scene_id = 4; // arbitrary integer

#define TESTOUT(NAME, P) {cout << NAME << P.x << ", " << P.y << ", " << P.z << endl;}

vector<glm::fvec3> model_pick_pts;
bool align_matching_model = false;
glm::fmat4x4 mat_match_model2ws;

void CallBackFunc_RsMouse(int event, int x, int y, int flags, void* userdata)
{
	OpttrkData& otrk_data = g_otrk_data;// *(opttrk_data*)userdata;

	vector<Point3f>& point3ds = otrk_data.calib_3d_pts;

#define MAX_POINTS_PAIRS 10
	static vector<glm::fvec3> pick_pts;
	static int spheres_id = 0;
	static int gathered_point_id = 0;

	if (!otrk_data.trk_info.is_updated) return;

	vzm::ObjStates sobj_state;
	sobj_state.color[3] = 1.0f;
	sobj_state.emission = 0.5f;
	sobj_state.diffusion = 0.5f;
	sobj_state.specular = 0.0f;

	if (rs_ms_mode == ADD_CALIB_POINTS)
	{
		if (event == EVENT_LBUTTONDOWN)
		{
			if (!otrk_data.trk_info.is_detected_probe) return;
			glm::fvec3 pt = otrk_data.trk_info.GetProbePinPoint();
			point3ds.push_back(Point3f(pt.x, pt.y, pt.z));

			cout << pt.x << ", " << pt.y << ", " << pt.z << endl;

			//vector<glm::fvec4> sphers_xyzr;
			//vector<glm::fvec3> sphers_rgb;
			//for (int i = 0; i < (int)point3ds.size(); i++)
			//{
			//	sphers_xyzr.push_back(glm::fvec4(point3ds[i].x, point3ds[i].y, point3ds[i].z, 0.01));
			//	sphers_rgb.push_back(glm::fvec3((i % otrk_data.cb_size.width) / (float)(otrk_data.cb_size.width - 1),
			//		(i / otrk_data.cb_size.width) / (float)(otrk_data.cb_size.height - 1), 1.f));
			//}
			//vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], point3ds.size(), otrk_data.cb_spheres_id);
			//vzm::ReplaceOrAddSceneObject(ws_scene_id, otrk_data.cb_spheres_id, otrk_data.obj_state);

			ofstream outfile(cb_positions);
			if (outfile.is_open())
			{
				outfile.clear();
				for (int i = 0; i < g_otrk_data.calib_3d_pts.size(); i++)
				{
					string line = to_string(g_otrk_data.calib_3d_pts[i].x) + " " +
						to_string(g_otrk_data.calib_3d_pts[i].y) + " " +
						to_string(g_otrk_data.calib_3d_pts[i].z);
					outfile << line << endl;
				}
			}
			outfile.close();
		}
	}
	else if (rs_ms_mode == GATHERING_POINTS)
	{
		if (flags & EVENT_FLAG_CTRLKEY)
		{
			if (pick_pts.size() == 0 || rs_pc_id == 0) return;
			if (event == EVENT_LBUTTONDOWN)
			{
				vzm::DeleteObject(gathered_point_id);
				vzm::ObjStates model_obj_state;
				vzm::GetSceneObjectState(ws_scene_id, rs_pc_id, model_obj_state);
				glm::fmat4x4 mat_ws2os = glm::inverse(*(glm::fmat4x4*)model_obj_state.os2ws);
				for (int i = 0; i < (int)pick_pts.size(); i++)
				{
					glm::fvec3 pos_pick_os = tr_pt(mat_ws2os, pick_pts[i]);
					vzmproc::GenerateSamplePoints(rs_pc_id, (float*)&pos_pick_os, 10.f, 0.3f, gathered_point_id);
				}

				vzm::ObjStates sobj_state;
				__cv4__ sobj_state.color = glm::fvec4(1, 0, 0, 1);
				sobj_state.emission = 0.5f;
				sobj_state.diffusion = 0.5f;
				sobj_state.specular = 0.0f;
				sobj_state.point_thickness = 5.f;
				*(glm::fmat4x4*)sobj_state.os2ws = *(glm::fmat4x4*)model_obj_state.os2ws;
				vzm::ReplaceOrAddSceneObject(ws_scene_id, gathered_point_id, sobj_state);
				vzm::ReplaceOrAddSceneObject(rs_scene_id, gathered_point_id, sobj_state);
			}
		}
		else if (flags & EVENT_FLAG_ALTKEY)
		{
			if (mesh_obj_id == 0) return;
			if (event == EVENT_LBUTTONDOWN)
			{
				// model's world to real world
				int num_crrpts = (int)min(model_pick_pts.size(), pick_pts.size());
				if (num_crrpts >= 3)
				{
					//for (int i = 0; i < num_crrpts; i++)
					//{
					//	cout << i << "crrs" << endl;
					//	TESTOUT("model_pick_pts : ", model_pick_pts[i]);
					//	TESTOUT("pick_pts : ", pick_pts[i]);
					//}
					glm::fmat4x4 mat_tr;
					if (helpers::ComputeRigidTransform(__FP model_pick_pts[0], __FP pick_pts[0], num_crrpts, __FP mat_tr[0]))
					{
						vzm::ObjStates model_obj_state;
						vzm::GetSceneObjectState(model_scene_id, mesh_obj_id, model_obj_state);

						mat_match_model2ws = mat_tr * (__cm4__ model_obj_state.os2ws);
						cout << "model matching done!" << endl;
						align_matching_model = true;
					}
				}
			}
			else if (event == EVENT_RBUTTONDOWN)
			{

			}
		}
		else
		{
			if (event == EVENT_LBUTTONDOWN || event == EVENT_RBUTTONDOWN)
			{
				if (event == EVENT_LBUTTONDOWN)
				{
					TESTOUT("world position : ", pos_probe_pin);
					pick_pts.push_back(pos_probe_pin);
				}
				else if (event == EVENT_RBUTTONDOWN)
				{
					if(pick_pts.size() > 0)
						pick_pts.pop_back();
				}
				if (pick_pts.size() > 0)
				{
					vector<glm::fvec4> spheres_xyzr;
					vector<glm::fvec3> spheres_rgb;
					for (int i = 0; i < (int)pick_pts.size(); i++)
					{
						glm::fvec4 sphere_xyzr = glm::fvec4(pick_pts[i], 0.005);
						spheres_xyzr.push_back(sphere_xyzr);
						glm::fvec3 sphere_rgb = glm::fvec3(0, 1, 0);
						spheres_rgb.push_back(sphere_rgb);
					}
					vzm::GenerateSpheresObject(__FP spheres_xyzr[0], __FP spheres_rgb[0], (int)pick_pts.size(), spheres_id);
					vzm::ReplaceOrAddSceneObject(ws_scene_id, spheres_id, sobj_state);
					vzm::ReplaceOrAddSceneObject(rs_scene_id, spheres_id, sobj_state);
				}
				else
				{
					vzm::DeleteObject(spheres_id);
					spheres_id = 0;
				}
			}
		}
	}
	else if (rs_ms_mode == PIN_ORIENTATION)
	{
		if (event == EVENT_LBUTTONDOWN)
		{
			if (is_calib_cam)
			{
				if (!otrk_data.trk_info.is_detected_sstool) return;

				glm::fvec3 pt = otrk_data.trk_info.GetProbePinPoint();

				glm::fmat4x4 mat_ws2tfrm = glm::inverse(otrk_data.trk_info.mat_tfrm2ws);
				ss_tool_info.pos_centers_tfrm.push_back(tr_pt(mat_ws2tfrm, pt));

				ofstream outfile(sst_positions);
				if (outfile.is_open())
				{
					outfile.clear();
					for (int i = 0; i < ss_tool_info.pos_centers_tfrm.size(); i++)
					{
						string line = to_string(ss_tool_info.pos_centers_tfrm[i].x) + " " +
							to_string(ss_tool_info.pos_centers_tfrm[i].y) + " " +
							to_string(ss_tool_info.pos_centers_tfrm[i].z);
						outfile << line << endl;
					}
				}
				outfile.close();
			}
		}
	}
}

void CallBackFunc_ModelMouse(int event, int x, int y, int flags, void* userdata)
{
	glm::ivec2 scene_cam = *(glm::ivec2*)userdata;
	vzm::CameraParameters cam_params;
	vzm::GetCameraParameters(scene_cam.x, cam_params, scene_cam.y);

	static vector<glm::fvec3> pick_pts;
	static int spheres_id = 0;
	static int gathered_point_id = 0;

	static int x_old = -1;
	static int y_old = -1;
	//if ((x - x_old) * (x - x_old) + (y - y_old) * (y - y_old) < 1) return;

	x_old = x;
	y_old = y;

	skip_main_thread = true;
	// https://docs.opencv.org/3.4/d7/dfc/group__highgui.html
	static helpers::arcball aball_ov;
	if (flags & EVENT_FLAG_CTRLKEY)
	{
		if (event == EVENT_LBUTTONUP || event == EVENT_RBUTTONUP)
		{
			if (mesh_obj_id == 0)
				Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "NO MESH!!");
			if (flags & EVENT_FLAG_CTRLKEY)
			{
				if (event == EVENT_LBUTTONUP)
				{
					int pick_obj = 0;
					glm::fvec3 pos_pick;
					vzm::PickObject(pick_obj, __FP pos_pick, x, y, scene_cam.x, scene_cam.y);
					if (pick_obj != 0)
					{
						cout << "picked : " << pick_obj << endl;
						TESTOUT("world position : ", pos_pick);
						pick_pts.push_back(pos_pick);
					}
				}
				else if (event == EVENT_RBUTTONUP)
				{
					if (pick_pts.size() > 0)
						pick_pts.pop_back();
				}
				if (pick_pts.size() > 0)
				{
					vector<glm::fvec4> spheres_xyzr;
					vector<glm::fvec3> spheres_rgb;
					for (int i = 0; i < (int)pick_pts.size(); i++)
					{
						glm::fvec4 sphere_xyzr = glm::fvec4(pick_pts[i], 0.001);
						spheres_xyzr.push_back(sphere_xyzr);
						glm::fvec3 sphere_rgb = glm::fvec3(1, 0, 0);
						spheres_rgb.push_back(sphere_rgb);
					}
					vzm::ObjStates sobj_state;
					sobj_state.color[3] = 1.0f;
					sobj_state.emission = 0.5f;
					sobj_state.diffusion = 0.5f;
					sobj_state.specular = 0.0f;
					vzm::GenerateSpheresObject(__FP spheres_xyzr[0], __FP spheres_rgb[0], (int)pick_pts.size(), spheres_id);
					vzm::ReplaceOrAddSceneObject(scene_cam.x, spheres_id, sobj_state);
				}
				else
				{
					vzm::DeleteObject(spheres_id);
					spheres_id = 0;
				}
				model_pick_pts = pick_pts;
				Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "Point : " + to_string((int)pick_pts.size()));
			}
		}
	}	
	else if (flags & EVENT_FLAG_ALTKEY)
	{
		if (pick_pts.size() == 0) return;
		if (event == EVENT_LBUTTONUP)
		{
			if (mesh_obj_id == 0)
				Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "NO MESH!!");
			vzm::DeleteObject(gathered_point_id);
			vzm::ObjStates model_obj_state;
			vzm::GetSceneObjectState(scene_cam.x, mesh_obj_id, model_obj_state);
			glm::fmat4x4 mat_ws2os = glm::inverse(*(glm::fmat4x4*)model_obj_state.os2ws);
			for (int i = 0; i < (int)pick_pts.size(); i++)
			{
				glm::fvec3 pos_pick_os = tr_pt(mat_ws2os, pick_pts[i]);
				vzmproc::GenerateSamplePoints(mesh_obj_id, (float*)&pos_pick_os, 10.f, 0.3f, gathered_point_id);
			}

			vzm::ObjStates sobj_state;
			__cv4__ sobj_state.color = glm::fvec4(1, 1, 0, 1);
			sobj_state.emission = 0.5f;
			sobj_state.diffusion = 0.5f;
			sobj_state.specular = 0.0f;
			sobj_state.point_thickness = 5.f;
			*(glm::fmat4x4*)sobj_state.os2ws = *(glm::fmat4x4*)model_obj_state.os2ws;
			vzm::ReplaceOrAddSceneObject(scene_cam.x, gathered_point_id, sobj_state);
			Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "Point : " + to_string((int)pick_pts.size()));
		}
	}
	else
	{
		// manipulating camera location
		if (event == EVENT_LBUTTONDOWN || event == EVENT_RBUTTONDOWN)
		{
			aball_ov.intializer((float*)&glm::fvec3(), 0.10f);

			helpers::cam_pose arc_cam_pose;
			glm::fvec3 pos = __cv3__ arc_cam_pose.pos = __cv3__ cam_params.pos;
			__cv3__ arc_cam_pose.up = __cv3__ cam_params.up;
			__cv3__ arc_cam_pose.view = __cv3__ cam_params.view;
			aball_ov.start((int*)&glm::ivec2(x, y), (float*)&glm::fvec2(cam_params.w, cam_params.h), arc_cam_pose);
		}
		else if (event == EVENT_MOUSEWHEEL)
		{
			if (getMouseWheelDelta(flags) > 0)
				__cv3__ cam_params.pos += 0.01f * (__cv3__ cam_params.view);
			else
				__cv3__ cam_params.pos -= 0.01f * (__cv3__ cam_params.view);
			vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
			Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "Point : " + to_string((int)pick_pts.size()));
		}
		else if (event == EVENT_MOUSEMOVE)
		{
			if (flags & EVENT_FLAG_LBUTTON)
			{
				helpers::cam_pose arc_cam_pose;
				aball_ov.pan_move((int*)&glm::ivec2(x, y), arc_cam_pose);
				__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
				__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
				__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
				vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
				Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "Point : " + to_string((int)pick_pts.size()));
			}
			else if (flags & EVENT_FLAG_RBUTTON)
			{
				helpers::cam_pose arc_cam_pose;
				aball_ov.move((int*)&glm::ivec2(x, y), arc_cam_pose);
				__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
				__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
				__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
				vzm::SetCameraParameters(scene_cam.x, cam_params, scene_cam.y);
				Show_Window_with_Texts(window_name_ms_view, scene_cam.x, scene_cam.y, "Point : " + to_string((int)pick_pts.size()));
			}
		}
	}

	//int key_pressed = cv::waitKey(10);
	skip_main_thread = false;
}

ArMarkerTracker ar_marker;
int main()
{
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

	vzm::LoadModelFile(model_path, mesh_obj_id);
	vzm::ValidatePickTarget(mesh_obj_id);
	int mesh_obj_ws_id = 0;
	vzm::GenerateCopiedObject(mesh_obj_id, mesh_obj_ws_id);

	vzm::CameraParameters cam_params;
	if (!optitrk::InitOptiTrackLib())
	{
		printf("Unable to license Motive API\n");
		return 1;
	}
	optitrk::LoadProfileAndCalibInfo(optrack_env, optrack_calib);
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
	vzm::SetCameraParameters(ws_scene_id, cam_params, ov_cam_id);

	int model_cam_id = 1; // arbitrary integer

	vzm::CameraParameters cam_params_model = cam_params;
	cam_params_model.np = 0.01f;
	cam_params_model.fp = 10.0f;
	__cv3__ cam_params_model.pos = glm::fvec3(0.3f, 0, 0);
	__cv3__ cam_params_model.up = glm::fvec3(0, 1.f, 0);
	__cv3__ cam_params_model.view = glm::fvec3(-1.f, 0, 0.f);
	vzm::SetCameraParameters(model_scene_id, cam_params_model, model_cam_id);

	vzm::SceneEnvParameters scn_env_params;
	scn_env_params.is_on_camera = false;
	scn_env_params.is_pointlight = false;
	scn_env_params.effect_ssao.is_on_ssao = false;
	scn_env_params.effect_ssao.kernel_r = 0.01f;
	scn_env_params.effect_ssao.num_dirs = 16;
	__cv3__ scn_env_params.pos_light = __cv3__ cam_params.pos;
	__cv3__ scn_env_params.dir_light = __cv3__ cam_params.view;
	vzm::SetSceneEnvParameters(ws_scene_id, scn_env_params);
	scn_env_params.is_on_camera = true;
	vzm::SetSceneEnvParameters(rs_scene_id, scn_env_params);
	vzm::SceneEnvParameters ms_scn_env_params = scn_env_params;
	ms_scn_env_params.is_on_camera = true;
	vzm::SetSceneEnvParameters(model_scene_id, ms_scn_env_params);

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
	//	vzm::ReplaceOrAddSceneObject(rs_scene_id, sp_obj_id, obj_state);
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
	cv::namedWindow(window_name_rs_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);
	cv::namedWindow(window_name_ws_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);
	cv::namedWindow(window_name_ms_view, WINDOW_NORMAL | WINDOW_AUTOSIZE);

	vzm::ObjStates model_state = obj_state;
	model_state.color[3] = 0.8;
	glm::fmat4x4 mat_s = glm::scale(glm::fvec3(0.001));
	__cm4__ model_state.os2ws = (__cm4__ model_state.os2ws) * mat_s;
	vzm::ReplaceOrAddSceneObject(model_scene_id, mesh_obj_id, model_state);
	Show_Window(window_name_ms_view, model_scene_id, model_cam_id);

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
	if(stream_rgb)
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
	vzm::SetCameraParameters(rs_scene_id, rs_cam_params, rs_cam_id);


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

	glm::ivec2 cb_data(ws_scene_id, ov_cam_id);
	cv::setMouseCallback(window_name_ws_view, CallBackFunc_WorldMouse, &cb_data);
	glm::ivec2 model_cb_data(model_scene_id, model_cam_id);
	cv::setMouseCallback(window_name_ms_view, CallBackFunc_ModelMouse, &model_cb_data);

	cv::setMouseCallback(window_name_rs_view, CallBackFunc_RsMouse, NULL);

	optitrk::UpdateFrame();
	glm::fmat4x4 mat_cam0_to_ws, mat_cam1_to_ws;
	optitrk::GetCameraLocation(0, (float*)&mat_cam0_to_ws);
	optitrk::GetCameraLocation(1, (float*)&mat_cam1_to_ws);

	Register_CamModel(ws_scene_id, mat_cam0_to_ws, "IR CAM 0", 0);
	Register_CamModel(ws_scene_id, mat_cam1_to_ws, "IR CAM 1", 1);

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


	// make 3d ui widgets
	int coord_grid_obj_id = 0, axis_lines_obj_id = 0, axis_texX_obj_id = 0, axis_texZ_obj_id = 0;
	World_GridAxis_Gen(coord_grid_obj_id, axis_lines_obj_id, axis_texX_obj_id, axis_texZ_obj_id);
	vzm::ObjStates grid_obj_state;
	grid_obj_state.color[3] = 0.7f;
	grid_obj_state.line_thickness = 0;
	vzm::ReplaceOrAddSceneObject(ws_scene_id, coord_grid_obj_id, grid_obj_state);
	bool foremost_surf_rendering = true;
	vzm::DebugTestSet("_bool_OnlyForemostSurfaces", &foremost_surf_rendering, sizeof(bool), ws_scene_id, ov_cam_id, coord_grid_obj_id);
	grid_obj_state.color[3] = 0.9f;
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_lines_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(1, 0.3, 0.3, 0.6);
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_texX_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(0.3, 0.3, 1, 0.6);
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_texZ_obj_id, grid_obj_state);

	int key_pressed = -1;
	bool recompile_hlsl = false;
	bool show_apis_console = false;
	while (key_pressed != 'q' && key_pressed != 27)
	{
		key_pressed = cv::waitKey(1);
		bool load_calib_points = false;
		switch (key_pressed) // http://www.asciitable.com/
		{
		case 91: postpone = max(postpone - 1, 0);  break; // [ 
		case 93: postpone += 1; break; // ] 
		case 97: use_new_version = false;  cout << "Use Prev Version : ON" << endl; break; // a 
		case 115: use_new_version = true; cout << "Use Prev Version : OFF" << endl; break; // s 
		case 100: recompile_hlsl = true; cout << "Recompile Shader!" << endl; break; // d
		case 108: load_calib_points = true; break; // l
		case 118: show_calib_frames = !show_calib_frames; break; // v
		case 112: show_pc = !show_pc; break; // p
		case 101: show_apis_console = !show_apis_console; break; // e
		case 99: calib_toggle = !calib_toggle; break; // c
		case 49: rs_ms_mode = RsMouseMode::NONE; break; // 1
		case 50: rs_ms_mode = RsMouseMode::ADD_CALIB_POINTS; break; // 2
		case 51: rs_ms_mode = RsMouseMode::GATHERING_POINTS; break; // 3
		case 52: rs_ms_mode = RsMouseMode::PIN_ORIENTATION; break; // 4
		}

		vzm::DisplayConsoleMessages(show_apis_console);

		if (skip_main_thread) continue;

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
			g_otrk_data.trk_info = trk_info;
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
				g_otrk_data.calib_3d_pts.clear();
				std::ifstream infile(cb_positions);
				string line;
				while (getline(infile, line))
				{
					std::istringstream iss(line);
					float a, b, c;
					if (!(iss >> a >> b >> c)) { break; } // error
					g_otrk_data.calib_3d_pts.push_back(Point3f(a, b, c));
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
			
			vector<glm::fvec4> sphers_xyzr;
			vector<glm::fvec3> sphers_rgb;

			auto marker_color = [](int idx, int w)
			{
				return glm::fvec3((idx % max(w, 1)) / (float)max(w - 1, 1), (idx / max(w, 1)) / (float)max(w - 1, 1), 1);
			};
			for (int i = 0; i < g_otrk_data.calib_3d_pts.size(); i++)
			{
				Point3f pt = g_otrk_data.calib_3d_pts[i];
				sphers_xyzr.push_back(glm::fvec4(pt.x, pt.y, pt.z, 0.01));
				sphers_rgb.push_back(marker_color(i, (int)g_otrk_data.calib_3d_pts.size() / 2));
			}


			if (sphers_xyzr.size() > 0)
			{
				vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0],
					g_otrk_data.calib_3d_pts.size(), g_otrk_data.cb_spheres_id);
				vzm::ReplaceOrAddSceneObject(ws_scene_id, g_otrk_data.cb_spheres_id, obj_state);
				vzm::ReplaceOrAddSceneObject(rs_scene_id, g_otrk_data.cb_spheres_id, obj_state);
			}
			else
			{
				vzm::ObjStates cstate = obj_state;
				cstate.is_visible = false;
				vzm::ReplaceOrAddSceneObject(ws_scene_id, g_otrk_data.cb_spheres_id, cstate);
				vzm::ReplaceOrAddSceneObject(rs_scene_id, g_otrk_data.cb_spheres_id, cstate);
			}

			static vector<int> calib_trial_cam_frame_ids;
			if (do_initialize_trk_points)
			{
				for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
					vzm::DeleteObject(calib_trial_cam_frame_ids[i]);
				calib_trial_cam_frame_ids.clear();
			}

			if (calib_toggle && trk_info.is_detected_rscam && g_otrk_data.calib_3d_pts.size() > 0)
			{
				// calibration routine
				Mat viewGray;
				cvtColor(imagebgr, viewGray, COLOR_BGR2GRAY);

				std::vector<__MarkerDetInfo> list_det_mks;
				ar_marker.track_markers(list_det_mks, viewGray.data, viewGray.cols, viewGray.rows, mk_ids);

				for (int i = 0; i < (int)list_det_mks.size(); i++)
				{
					__MarkerDetInfo& armk = list_det_mks[i];

					int id = armk.id;
					glm::fvec3 _rgb = marker_color(id - 1, g_otrk_data.calib_3d_pts.size() / 2);

					for (int j = 0; j < 4; j++)
						circle(imagebgr, Point(armk.corners2d[2 * j + 0], armk.corners2d[2 * j + 1]), 1, CV_RGB(_rgb.r * 255, _rgb.g * 255, _rgb.b * 255), 2);
				}

				if (list_det_mks.size() > 0)
				{
					vector<Point2f> point2d;
					vector<Point3f> point3d;

					for (int i = 0; i < (int)list_det_mks.size(); i++)
					{
						__MarkerDetInfo& armk = list_det_mks[i];
						if (armk.id > g_otrk_data.calib_3d_pts.size()) continue;

						Point2f pt2d = Point2f(0, 0);
						vector<float>& cpts = armk.corners2d;
						for (int k = 0; k < 4; k++)
							pt2d += Point2f(cpts[k * 2 + 0], cpts[k * 2 + 1]);

						point2d.push_back(pt2d / 4.f);
						point3d.push_back(g_otrk_data.calib_3d_pts[armk.id - 1]);
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
							is_calib_cam = true;
							num_calib++;
						}

						for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
						{
							vzm::ObjStates cstate;
							vzm::GetSceneObjectState(ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
							cstate.is_visible = true;
							vzm::ReplaceOrAddSceneObject(ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
						}

						int calib_frame_id = 0;
						Axis_Gen(mat_clf2ws, 0.05f, calib_frame_id);
						vzm::ObjStates cstate = obj_state;
						cstate.color[3] = 0.3f;
						vzm::ReplaceOrAddSceneObject(ws_scene_id, calib_frame_id, cstate);
						calib_trial_cam_frame_ids.push_back(calib_frame_id);
					}
					do_initialize_trk_points = false;
				}
			}
			if(!show_calib_frames)
			{
				for (int i = 0; i < calib_trial_cam_frame_ids.size(); i++)
				{
					vzm::ObjStates cstate;
					vzm::GetSceneObjectState(ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
					cstate.is_visible = false;
					vzm::ReplaceOrAddSceneObject(ws_scene_id, calib_trial_cam_frame_ids[i], cstate);
				}
			}
			if(show_pc)
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
					vzm::GetCameraParameters(ws_scene_id, _cam_params, ov_cam_id);

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
						if(normalmap) nrl_pts[i] = normalmap[i % _w + (i / _w) * _w];
					}
					if (normalmap) delete[] normalmap;
					//vzm::GeneratePointCloudObject(__FP pos_pts[0], NULL, __FP color_pts[0], (int)points.size(), rs_pc_id);
					vzm::GeneratePointCloudObject(__FP pos_pts[0], __FP nrl_pts[0], NULL, (int)points.size(), rs_pc_id);
					vzm::ReplaceOrAddSceneObject(ws_scene_id, rs_pc_id, obj_state_pts);
					vzm::ReplaceOrAddSceneObject(rs_scene_id, rs_pc_id, obj_state_pts);
					vzm::DebugTestSet("_bool_OnlyForemostSurfaces", &foremost_surf_rendering, sizeof(bool), ws_scene_id, ov_cam_id, rs_pc_id);
				}
			}
			else
			{
				vzm::ObjStates obj_state_pts;
				vzm::GetSceneObjectState(ws_scene_id, rs_pc_id, obj_state_pts);
				obj_state_pts.is_visible = false;
				vzm::ReplaceOrAddSceneObject(ws_scene_id, rs_pc_id, obj_state_pts);
				vzm::ReplaceOrAddSceneObject(rs_scene_id, rs_pc_id, obj_state_pts);
			}


			if(is_calib_cam)
			{
				pos_probe_pin = tr_pt(trk_info.mat_probe2ws, glm::fvec3());
				glm::fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;

				//rs_cam_tris_id, rs_cam_lines_id, rs_cam_txt_id
				if(trk_info.is_detected_rscam)
					Register_CamModel(ws_scene_id, mat_rscs2ws, "RS CAM 0", 2);
				
				auto register_mks = [&obj_state](const glm::fvec3* pos_list, const int num_mks, const float r)
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

					if (num_mks > 0)
					{
						vzm::GenerateSpheresObject(__FP sphers_xyzr[0], __FP sphers_rgb[0], num_mks, obj_mks_id);
						vzm::ReplaceOrAddSceneObject(ws_scene_id, obj_mks_id, obj_state);
						vzm::ReplaceOrAddSceneObject(rs_scene_id, obj_mks_id, obj_state);
					}
					else
					{
						vzm::ObjStates cstate = obj_state;
						cstate.is_visible = false;
						vzm::ReplaceOrAddSceneObject(ws_scene_id, obj_mks_id, cstate);
						vzm::ReplaceOrAddSceneObject(rs_scene_id, obj_mks_id, cstate);
					}
				};
				register_mks((glm::fvec3*)&trk_info.mk_xyz_list[0], trk_info.mk_xyz_list.size() / 3, 0.005);

				//if (!trk_info.is_detected_rscam)
					// to do // via SLAM (with AR mks?!)
			}

			if (trk_info.mk_residue_list.size() > 5)
			{
				for (int i = 0; i < trk_info.mk_residue_list.size(); i++)
				{
					//cout << i << " ==> " << trk_info.mk_residue_list[i] << endl;
				}
			}

			auto set_rb_axis = [&obj_state](const bool is_detected, const glm::fmat4x4& mat_frm2ws, int& obj_id)
			{
				if (is_detected)
				{
					Axis_Gen(mat_frm2ws, 0.07, obj_id);
					vzm::ReplaceOrAddSceneObject(ws_scene_id, obj_id, obj_state);
				}
				else if (obj_id != 0)
				{
					vzm::ObjStates ostate = obj_state;
					ostate.is_visible = false;
					vzm::ReplaceOrAddSceneObject(ws_scene_id, obj_id, ostate);
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
					vzm::ReplaceOrAddSceneObject(ws_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
					vzm::ReplaceOrAddSceneObject(rs_scene_id, ss_tool_info.ss_tool_guide_points_id, cobjstate);
				}
			}

			if (trk_info.is_detected_sshead)
			{
				static glm::fmat4x4 mat_os2headfrm;
				vzm::ObjStates model_obj_state;
				vzm::GetSceneObjectState(model_scene_id, mesh_obj_id, model_obj_state);

				if (align_matching_model)
				{
					cout << "register rigid model!" << endl;
					glm::fmat4x4 mat_ws2headfrm = glm::inverse(trk_info.mat_headfrm2ws);
					mat_os2headfrm = mat_ws2headfrm * mat_match_model2ws;
					align_matching_model = false;
				}

				__cm4__ model_obj_state.os2ws = trk_info.mat_headfrm2ws * mat_os2headfrm;

				// REFACTORING 필요!!!!
				//SetTransformMatrixOS2WS 을 SCENE PARAM 으로 바꾸기!
				vzm::ReplaceOrAddSceneObject(ws_scene_id, mesh_obj_ws_id, model_obj_state);
				vzm::ReplaceOrAddSceneObject(rs_scene_id, mesh_obj_ws_id, model_obj_state);
			}

			vzm::CameraParameters _rs_cam_params;
			vzm::GetCameraParameters(rs_scene_id, _rs_cam_params, rs_cam_id);
			ComputeCameraStates(mat_rscs2clf, mat_clf2ws, _rs_cam_params);
			vzm::SetCameraParameters(rs_scene_id, _rs_cam_params, rs_cam_id);

			scn_env_params.is_on_camera = false;
			__cv3__ scn_env_params.pos_light = __cv3__ _rs_cam_params.pos;
			__cv3__ scn_env_params.dir_light = __cv3__ _rs_cam_params.view;
			vzm::SetSceneEnvParameters(ws_scene_id, scn_env_params);
			
			vzm::RenderScene(rs_scene_id, rs_cam_id);
			unsigned char* ptr_rgba;
			float* ptr_zdepth;
			int rs_w, rs_h;
			if (vzm::GetRenderBufferPtrs(rs_scene_id, &ptr_rgba, &ptr_zdepth, &rs_w, &rs_h, rs_cam_id))
				copy_back_ui_buffer(imagebgr.data, ptr_rgba, rs_w, rs_h, false);
			
			cv::putText(imagebgr, "PnP reprojection error : " + to_string(pnp_err) + " pixels, # samples : " + to_string(calib_samples),
				cv::Point(3, 25), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			cv::putText(imagebgr, "Calibration Points : " + to_string(g_otrk_data.calib_3d_pts.size()),
				cv::Point(3, 50), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(185, 255, 255));
			cv::putText(imagebgr, "# of calibrations : " + to_string(num_calib),
				cv::Point(3, 75), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			cv::putText(imagebgr, "mouse mode : " + EtoString(rs_ms_mode), cv::Point(3, 150), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			string b_calib_toggle = calib_toggle ? "true" : "false";
			cv::putText(imagebgr, "Calibration Toggle : " + b_calib_toggle + ", Postpone : " + to_string(postpone) + " ms",
				cv::Point(3, 100), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));
			
			if (is_calib_cam && !trk_info.is_detected_rscam)
				cv::putText(imagebgr, "RS Cam is out of tracking volume !!", cv::Point(400, 50), cv::FONT_HERSHEY_DUPLEX, 2.0, CV_RGB(255, 0, 0), 3, LineTypes::LINE_AA);

			imshow(window_name_rs_view, imagebgr);
			/**/
		}

		vzm::DebugTestSet("_bool_ReloadHLSLObjFiles", &recompile_hlsl, sizeof(bool), -1, -1);
		vzm::DebugTestSet("_bool_TestOit", &use_new_version, sizeof(bool), -1, -1);
		Show_Window(window_name_ws_view, ws_scene_id, ov_cam_id);
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