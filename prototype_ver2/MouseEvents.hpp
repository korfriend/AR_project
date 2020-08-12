#pragma once

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <fstream>
#include <sstream>
#include <iostream>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#include "VisMtvApi.h"
#include "../optitrk/optitrack.h"
#include "../kar_helpers.hpp"

using namespace std;
using namespace cv;

struct EventGlobalInfo
{
	int scene_id;
	int cam_id;
	GlobalInfo& ginfo;
	EventGlobalInfo(GlobalInfo& global_info, int _scene_id, int _cam_id) : ginfo(global_info), scene_id(_scene_id), cam_id(_cam_id) {}
};

void CallBackFunc_WorldMouse(int event, int x, int y, int flags, void* userdata)
{
	EventGlobalInfo* eginfo = (EventGlobalInfo*)userdata;

	vzm::CameraParameters cam_params;
	vzm::GetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);

	// https://docs.opencv.org/3.4/d7/dfc/group__highgui.html
	static helpers::arcball aball_ov;
	if (event == EVENT_LBUTTONDOWN || event == EVENT_RBUTTONDOWN)
	{
		if (flags & EVENT_FLAG_CTRLKEY && eginfo->ginfo.manual_set_mode == MsMouseMode::ADD_CALIB_POINTS)
		{
			vector<Point3f>& point3ds = eginfo->ginfo.otrk_data.calib_3d_pts;
			if (event == EVENT_LBUTTONDOWN)
			{
				int pick_obj = 0;
				glm::fvec3 pos_pick;
				vzm::PickObject(pick_obj, __FP pos_pick, x, y, eginfo->scene_id, eginfo->cam_id);
				cout << "PICK ID : " << pick_obj << endl;

				if (pick_obj != 0)
				{
					glm::fvec3 mk_pt = eginfo->ginfo.vzmobjid2mkid[pick_obj];
					cout << "----> " << eginfo->ginfo.vzmobjid2mkid.size() << endl;
					TESTOUT("==> ", mk_pt);
					if (mk_pt != glm::fvec3(0))
					{
						const float zig_hight = 0.03;
						const float mk_r = 0.009;

						glm::fvec3 pt = mk_pt - glm::fvec3(0, 1, 0) * (zig_hight + mk_r);
						TESTOUT("mk position " + to_string(point3ds.size()), pt);
						point3ds.push_back(Point3f(pt.x, pt.y, pt.z));
					}
				}
			}
			else
			{
				if (point3ds.size() > 0)
					point3ds.pop_back();
			}

			ofstream outfile(eginfo->ginfo.cb_positions);
			if (outfile.is_open())
			{
				outfile.clear();
				for (int i = 0; i < point3ds.size(); i++)
				{
					string line = to_string(point3ds[i].x) + " " +
						to_string(point3ds[i].y) + " " +
						to_string(point3ds[i].z);
					outfile << line << endl;
				}
			}
			outfile.close();
		}
		else
		{
			aball_ov.intializer((float*)&glm::fvec3(), 2.0f);

			helpers::cam_pose arc_cam_pose;
			glm::fvec3 pos = __cv3__ arc_cam_pose.pos = __cv3__ cam_params.pos;
			__cv3__ arc_cam_pose.up = __cv3__ cam_params.up;
			__cv3__ arc_cam_pose.view = __cv3__ cam_params.view;
			aball_ov.start((int*)&glm::ivec2(x, y), (float*)&glm::fvec2(cam_params.w, cam_params.h), arc_cam_pose);
		}
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
		vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
	}
	else if (event == EVENT_MOUSEMOVE && !(flags & EVENT_FLAG_CTRLKEY))
	{
		if (flags & EVENT_FLAG_LBUTTON)
		{
			helpers::cam_pose arc_cam_pose;
			aball_ov.pan_move((int*)&glm::ivec2(x, y), arc_cam_pose);
			__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
			__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
			__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
			vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
		}
		else if (flags & EVENT_FLAG_RBUTTON)
		{
			helpers::cam_pose arc_cam_pose;
			aball_ov.move((int*)&glm::ivec2(x, y), arc_cam_pose);
			__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
			__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
			__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
			vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
		}
	}
}

void CallBackFunc_RsMouse(int event, int x, int y, int flags, void* userdata)
{
	EventGlobalInfo* eginfo = (EventGlobalInfo*)userdata;
	OpttrkData& otrk_data = eginfo->ginfo.otrk_data;// *(opttrk_data*)userdata;

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

	if (eginfo->ginfo.manual_set_mode == ADD_CALIB_POINTS)
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

			ofstream outfile(eginfo->ginfo.cb_positions);
			if (outfile.is_open())
			{
				outfile.clear();
				for (int i = 0; i < eginfo->ginfo.otrk_data.calib_3d_pts.size(); i++)
				{
					string line = to_string(eginfo->ginfo.otrk_data.calib_3d_pts[i].x) + " " +
						to_string(eginfo->ginfo.otrk_data.calib_3d_pts[i].y) + " " +
						to_string(eginfo->ginfo.otrk_data.calib_3d_pts[i].z);
					outfile << line << endl;
				}
			}
			outfile.close();
		}
	}
	else if (eginfo->ginfo.manual_set_mode == GATHERING_POINTS)
	{
		if (flags & EVENT_FLAG_CTRLKEY)
		{
			if (pick_pts.size() == 0 || eginfo->ginfo.rs_pc_id == 0) return;
			if (event == EVENT_LBUTTONDOWN)
			{
				vzm::DeleteObject(gathered_point_id);
				vzm::ObjStates model_obj_state;
				vzm::GetSceneObjectState(eginfo->ginfo.ws_scene_id, eginfo->ginfo.rs_pc_id, model_obj_state);
				glm::fmat4x4 mat_ws2os = glm::inverse(*(glm::fmat4x4*)model_obj_state.os2ws);
				for (int i = 0; i < (int)pick_pts.size(); i++)
				{
					glm::fvec3 pos_pick_os = tr_pt(mat_ws2os, pick_pts[i]);
					vzmproc::GenerateSamplePoints(eginfo->ginfo.rs_pc_id, (float*)&pos_pick_os, 10.f, 0.3f, gathered_point_id);
				}

				vzm::ObjStates sobj_state;
				__cv4__ sobj_state.color = glm::fvec4(1, 0, 0, 1);
				sobj_state.emission = 0.5f;
				sobj_state.diffusion = 0.5f;
				sobj_state.specular = 0.0f;
				sobj_state.point_thickness = 5.f;
				*(glm::fmat4x4*)sobj_state.os2ws = *(glm::fmat4x4*)model_obj_state.os2ws;
				vzm::ReplaceOrAddSceneObject(eginfo->ginfo.ws_scene_id, gathered_point_id, sobj_state);
				vzm::ReplaceOrAddSceneObject(eginfo->ginfo.rs_scene_id, gathered_point_id, sobj_state);
			}
		}
		else if (flags & EVENT_FLAG_ALTKEY)
		{
			if (eginfo->ginfo.model_obj_id == 0) return;
			if (event == EVENT_LBUTTONDOWN)
			{
				// model's world to real world
				int num_crrpts = (int)min(eginfo->ginfo.model_pick_pts.size(), pick_pts.size());
				if (num_crrpts >= 3)
				{
					//for (int i = 0; i < num_crrpts; i++)
					//{
					//	cout << i << "crrs" << endl;
					//	TESTOUT("eginfo->ginfo.model_pick_pts : ", eginfo->ginfo.model_pick_pts[i]);
					//	TESTOUT("pick_pts : ", pick_pts[i]);
					//}
					glm::fmat4x4 mat_tr;
					if (helpers::ComputeRigidTransform(__FP eginfo->ginfo.model_pick_pts[0], __FP pick_pts[0], num_crrpts, __FP mat_tr[0]))
					{
						vzm::ObjStates model_obj_state;
						vzm::GetSceneObjectState(eginfo->ginfo.model_scene_id, eginfo->ginfo.model_obj_id, model_obj_state);

						eginfo->ginfo.mat_match_model2ws = mat_tr * (__cm4__ model_obj_state.os2ws);
						cout << "model matching done!" << endl;
						eginfo->ginfo.align_matching_model = true;
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
					TESTOUT("world position : ", eginfo->ginfo.pos_probe_pin);
					pick_pts.push_back(eginfo->ginfo.pos_probe_pin);
				}
				else if (event == EVENT_RBUTTONDOWN)
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
						glm::fvec4 sphere_xyzr = glm::fvec4(pick_pts[i], 0.005);
						spheres_xyzr.push_back(sphere_xyzr);
						glm::fvec3 sphere_rgb = glm::fvec3(0, 1, 0);
						spheres_rgb.push_back(sphere_rgb);
					}
					vzm::GenerateSpheresObject(__FP spheres_xyzr[0], __FP spheres_rgb[0], (int)pick_pts.size(), spheres_id);
					vzm::ReplaceOrAddSceneObject(eginfo->ginfo.ws_scene_id, spheres_id, sobj_state);
					vzm::ReplaceOrAddSceneObject(eginfo->ginfo.rs_scene_id, spheres_id, sobj_state);
				}
				else
				{
					vzm::DeleteObject(spheres_id);
					spheres_id = 0;
				}
			}
		}
	}
	//else if (eginfo->ginfo.manual_set_mode == PIN_ORIENTATION)
	//{
	//	if (event == EVENT_LBUTTONDOWN)
	//	{
	//		if (eginfo->ginfo.is_calib_cam)
	//		{
	//			if (!otrk_data.trk_info.is_detected_sstool) return;
	//
	//			glm::fvec3 pt = otrk_data.trk_info.GetProbePinPoint();
	//
	//			glm::fmat4x4 mat_ws2tfrm = glm::inverse(otrk_data.trk_info.mat_tfrm2ws);
	//			ss_tool_info.pos_centers_tfrm.push_back(tr_pt(mat_ws2tfrm, pt));
	//
	//			ofstream outfile(eginfo->ginfo.sst_positions);
	//			if (outfile.is_open())
	//			{
	//				outfile.clear();
	//				for (int i = 0; i < ss_tool_info.pos_centers_tfrm.size(); i++)
	//				{
	//					string line = to_string(ss_tool_info.pos_centers_tfrm[i].x) + " " +
	//						to_string(ss_tool_info.pos_centers_tfrm[i].y) + " " +
	//						to_string(ss_tool_info.pos_centers_tfrm[i].z);
	//					outfile << line << endl;
	//				}
	//			}
	//			outfile.close();
	//		}
	//	}
	//}
}

void CallBackFunc_ModelMouse(int event, int x, int y, int flags, void* userdata)
{
	EventGlobalInfo* eginfo = (EventGlobalInfo*)userdata;
	vzm::CameraParameters cam_params;
	vzm::GetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);

	static vector<glm::fvec3> pick_pts;
	static int spheres_id = 0;
	static int gathered_point_id = 0;

	static int x_old = -1;
	static int y_old = -1;
	//if ((x - x_old) * (x - x_old) + (y - y_old) * (y - y_old) < 1) return;

	x_old = x;
	y_old = y;

	eginfo->ginfo.skip_main_thread = true;
	// https://docs.opencv.org/3.4/d7/dfc/group__highgui.html
	static helpers::arcball aball_ov;
	if (flags & EVENT_FLAG_CTRLKEY)
	{
		if (event == EVENT_LBUTTONUP || event == EVENT_RBUTTONUP)
		{
			if (eginfo->ginfo.model_obj_id == 0)
				Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "NO MESH!!");
			if (flags & EVENT_FLAG_CTRLKEY)
			{
				if (event == EVENT_LBUTTONUP)
				{
					int pick_obj = 0;
					glm::fvec3 pos_pick;
					vzm::PickObject(pick_obj, __FP pos_pick, x, y, eginfo->scene_id, eginfo->cam_id);
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
					vzm::ReplaceOrAddSceneObject(eginfo->scene_id, spheres_id, sobj_state);
				}
				else
				{
					vzm::DeleteObject(spheres_id);
					spheres_id = 0;
				}
				eginfo->ginfo.model_pick_pts = pick_pts;
				Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "Point : " + to_string((int)pick_pts.size()));
			}
		}
	}
	else if (flags & EVENT_FLAG_ALTKEY)
	{
		if (pick_pts.size() == 0) return;
		if (event == EVENT_LBUTTONUP)
		{
			if (eginfo->ginfo.model_obj_id == 0)
				Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "NO MESH!!");
			vzm::DeleteObject(gathered_point_id);
			vzm::ObjStates model_obj_state;
			vzm::GetSceneObjectState(eginfo->scene_id, eginfo->ginfo.model_obj_id, model_obj_state);
			glm::fmat4x4 mat_ws2os = glm::inverse(*(glm::fmat4x4*)model_obj_state.os2ws);
			for (int i = 0; i < (int)pick_pts.size(); i++)
			{
				glm::fvec3 pos_pick_os = tr_pt(mat_ws2os, pick_pts[i]);
				vzmproc::GenerateSamplePoints(eginfo->ginfo.model_obj_id, (float*)&pos_pick_os, 10.f, 0.3f, gathered_point_id);
			}

			vzm::ObjStates sobj_state;
			__cv4__ sobj_state.color = glm::fvec4(1, 1, 0, 1);
			sobj_state.emission = 0.5f;
			sobj_state.diffusion = 0.5f;
			sobj_state.specular = 0.0f;
			sobj_state.point_thickness = 5.f;
			*(glm::fmat4x4*)sobj_state.os2ws = *(glm::fmat4x4*)model_obj_state.os2ws;
			vzm::ReplaceOrAddSceneObject(eginfo->scene_id, gathered_point_id, sobj_state);
			Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "Point : " + to_string((int)pick_pts.size()));
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
			vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
			Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "Point : " + to_string((int)pick_pts.size()));
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
				vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
				Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "Point : " + to_string((int)pick_pts.size()));
			}
			else if (flags & EVENT_FLAG_RBUTTON)
			{
				helpers::cam_pose arc_cam_pose;
				aball_ov.move((int*)&glm::ivec2(x, y), arc_cam_pose);
				__cv3__ cam_params.pos = __cv3__ arc_cam_pose.pos;
				__cv3__ cam_params.up = __cv3__ arc_cam_pose.up;
				__cv3__ cam_params.view = __cv3__ arc_cam_pose.view;
				vzm::SetCameraParameters(eginfo->scene_id, cam_params, eginfo->cam_id);
				Show_Window_with_Texts(eginfo->ginfo.window_name_ms_view, eginfo->scene_id, eginfo->cam_id, "Point : " + to_string((int)pick_pts.size()));
			}
		}
	}

	//int key_pressed = cv::waitKey(10);
	eginfo->ginfo.skip_main_thread = false;
}