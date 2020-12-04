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

int zoom_scene_id;
int zoom_cam_id;
string window_name_zs_view;
int zoom_w;
int zoom_h;

int zoom_scene_stg_id;
int zoom_cam_stg_id;
int zoom_stg_w;
int zoom_stg_h;

int brain_ms_obj_id = 0;
int brain_ws_obj_id = 0;
int ventricle_ms_obj_id = 0;
int ventricle_ws_obj_id = 0;

std::vector<int> guide_line_ids;		// 추후에 global로 들어올 것
std::vector<glm::fvec3> guide_lines;	// 추후에 global로 들어올 것

void InitializeVarSettings(GlobalInfo& ginfo)
{
	var_settings::InitializeVarSettings(0, true, "marker");

	// SSU ////////////////////////////////////////////////////////////////////////////////////
	zoom_scene_id = 6;
	zoom_cam_id = 1;
	zoom_w = 300;
	zoom_h = 300;
	window_name_zs_view = "Zoom View";

	zoom_scene_stg_id = 7;
	zoom_cam_stg_id = 1;
	zoom_stg_w = 300;
	zoom_stg_h = 300;
}
void SetCvWindows(GlobalInfo& ginfo)
{
	var_settings::SetCvWindows();
	//var_settings::GetVarInfo(&ginfo);

	// SSU //////////////////////////////////////////////////////////////////
	cv::namedWindow(window_name_zs_view, WINDOW_AUTOSIZE);
	//cv::namedWindow("zs_mirror", WINDOW_NORMAL);
	//Show_Window(window_name_zs_view, zoom_scene_id, zoom_cam_id);
}
void SetPreoperations(GlobalInfo& ginfo, const int rs_w, const int rs_h, const int ws_w, const int ws_h, const int stg_w, const int stg_h, const int eye_w, const int eye_h)
{
	var_settings::SetPreoperations(rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);

	// SSU ////////////////////////////////////////////////////////////////////////////////////
	int ov_cam_id = var_settings::GetCameraID_SSU(ginfo.ws_scene_id);

	// load model //
	string brainPath = "..\\Data\\brain.obj";
	vzm::LoadModelFile(brainPath, brain_ms_obj_id);
	vzm::GenerateCopiedObject(brain_ms_obj_id, brain_ws_obj_id);			// copy

	string ventriclePath = "..\\Data\\ventricle.obj";
	vzm::LoadModelFile(ventriclePath, ventricle_ms_obj_id);
	vzm::GenerateCopiedObject(ventricle_ms_obj_id, ventricle_ws_obj_id);	// copy


	// zoom cam, scene (realsense, smartglass) //
	vzm::CameraParameters zoom_cam_params;
	vzm::GetCameraParameters(ginfo.ws_scene_id, zoom_cam_params, ov_cam_id);			// copy

	zoom_cam_params.w = zoom_w;
	zoom_cam_params.h = zoom_h;
	zoom_cam_params.ip_h = zoom_cam_params.ip_w;
	vzm::SetCameraParameters(zoom_scene_id, zoom_cam_params, zoom_cam_id);

	vzm::SceneEnvParameters zoom_scn_env_params;
	vzm::GetSceneEnvParameters(ginfo.ws_scene_id, zoom_scn_env_params);				// copy
	zoom_scn_env_params.is_on_camera = true;
	vzm::SetSceneEnvParameters(zoom_scene_id, zoom_scn_env_params);

	vzm::SetCameraParameters(zoom_scene_stg_id, zoom_cam_params, zoom_cam_stg_id);
	vzm::SetSceneEnvParameters(zoom_scene_stg_id, zoom_scn_env_params);


	// zoom scene grid
	GenWorldGrid(ginfo.zoom_scene_id, zoom_cam_id);
}
void LoadPresets(GlobalInfo& ginfo, const std::string& probe_specifier_rb_name)
{
	std::string preset_path = var_settings::GetDefaultFilePath();
	ginfo.custom_pos_file_paths["ss_tool_v2"] = preset_path + "..\\Preset\\ss_tool_v2_se.txt";

	var_settings::LoadPresets();
}
void DeinitializeVarSettings(GlobalInfo& ginfo)
{
	var_settings::DeinitializeVarSettings();
	var_settings::GetVarInfo(&ginfo);
}

void UpdateModel(GlobalInfo& ginfo, Simulation& s, bool bDrawWireFrame)
{
	if (ginfo.is_modelaligned) {
		// deform 반영 ////////////////////////////////////////////////////////////////////////////////////////////
		glm::fvec3 *pos_xyz_list, *nrl_xyz_list;
		unsigned int* idx_prims;
		int num_vtx, num_prims, stride_idx;
		glm::fmat3x3 mat_s;

		// brain
		vzm::GetPModelData(brain_ws_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);

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
		vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, brain_ws_obj_id);
		delete[] pos_xyz_list;
		delete[] nrl_xyz_list;
		delete[] idx_prims;

		// ventricle
		vzm::GetPModelData(ventricle_ws_obj_id, (float**)&pos_xyz_list, (float**)&nrl_xyz_list, nullptr, nullptr, num_vtx, &idx_prims, num_prims, stride_idx);
		for (int c = 0; c < s.softBodies[0]->m_child.size(); c++) {
			for (int i = 0, ni = s.softBodies[0]->m_child[c]->m_surfaceMeshFace.size(); i < ni; i++) {
				const CiSoftBody::Face&	f = s.softBodies[0]->m_child[c]->m_surfaceMeshFace[i];
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
		vzm::GeneratePrimitiveObject((float*)pos_xyz_list, (float*)nrl_xyz_list, NULL, NULL, num_vtx, idx_prims, num_prims, stride_idx, ventricle_ws_obj_id);
		delete[] pos_xyz_list;
		delete[] nrl_xyz_list;
		delete[] idx_prims;

		// rendering ////////////////////////////////////////////////////////////////////////////////////////////
		// realsense scene (20201111 - 현재 rs scene state에 ghost effect가 적용되어 있음)
		vzm::ObjStates model_rs_states, brain_rs_states, ventricle_rs_states;
		vzm::GetSceneObjectState(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_rs_states);

		model_rs_states.color[0] = 1.0; model_rs_states.color[1] = 1.0; model_rs_states.color[2] = 1.0;
		model_rs_states.color[3] = 1;// 0.1;

		brain_rs_states = model_rs_states;
		if (bDrawWireFrame) {
			brain_rs_states.is_wireframe = true;
		}
		else {
			brain_rs_states.is_wireframe = false;
		}
		brain_rs_states.wire_color[0] = 0.5; brain_rs_states.wire_color[1] = 0.5; brain_rs_states.wire_color[2] = 0.5; brain_rs_states.wire_color[3] = 1.0;
		brain_rs_states.color[0] = 0.5; brain_rs_states.color[1] = 0.5; brain_rs_states.color[2] = 0.5; brain_rs_states.color[3] = 1.0;

		ventricle_rs_states = model_rs_states;
		if (bDrawWireFrame) {
			ventricle_rs_states.is_wireframe = true;
		}
		else {
			ventricle_rs_states.is_wireframe = false;
		}
		ventricle_rs_states.wire_color[0] = 0.9;	ventricle_rs_states.wire_color[1] = 0.5;	ventricle_rs_states.wire_color[2] = 0.5; ventricle_rs_states.wire_color[3] = 1.0;
		ventricle_rs_states.color[0] = 1.0; ventricle_rs_states.color[1] = 0; ventricle_rs_states.color[2] = 0; ventricle_rs_states.color[3] = 1.0;

		vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ginfo.model_ws_obj_id, model_rs_states);
		vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, brain_ws_obj_id, brain_rs_states);
		vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, ventricle_ws_obj_id, ventricle_rs_states);

		// world scene
		vzm::ObjStates model_ws_states, brain_ws_states, ventricle_ws_states;
		memcpy(model_ws_states.os2ws, model_rs_states.os2ws, sizeof(float) * 16);
		model_ws_states.color[3] = 0.1;// 0.1;

		brain_ws_states = model_ws_states;
		if (bDrawWireFrame) {
			brain_ws_states.is_wireframe = true;
		}
		else {
			brain_ws_states.is_wireframe = false;
		}
		brain_ws_states.wire_color[0] = 0.5; brain_ws_states.wire_color[1] = 0.5; brain_ws_states.wire_color[2] = 0.5; brain_ws_states.wire_color[3] = 0.2;
		brain_ws_states.color[0] = 0.5; brain_ws_states.color[1] = 0.5; brain_ws_states.color[2] = 0.5; brain_ws_states.color[3] = 0.3;

		ventricle_ws_states = model_ws_states;
		if (bDrawWireFrame) {
			ventricle_ws_states.is_wireframe = true;
		}
		else {
			ventricle_ws_states.is_wireframe = false;
		}
		ventricle_ws_states.wire_color[0] = 0.9;	ventricle_ws_states.wire_color[1] = 0.5;	ventricle_ws_states.wire_color[2] = 0.5; ventricle_ws_states.wire_color[3] = 0.2;
		ventricle_ws_states.color[0] = 1.0; ventricle_ws_states.color[1] = 0; ventricle_ws_states.color[2] = 0; ventricle_ws_states.color[3] = 1.0;

		vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ginfo.model_ws_obj_id, model_ws_states);
		vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, brain_ws_obj_id, brain_ws_states);
		vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, ventricle_ws_obj_id, ventricle_ws_states);

		// smartglass scene
		vzm::ObjStates model_stg_states, brain_stg_states, ventricle_stg_states;
		model_stg_states = model_ws_states;
		brain_stg_states = brain_ws_states;
		ventricle_stg_states = ventricle_ws_states;

		model_stg_states.color[3] = 0.5;
		brain_stg_states.color[3] = 0.8;
		ventricle_stg_states.color[3] = 1.0;

		vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, ginfo.model_ws_obj_id, model_stg_states);
		vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, brain_ws_obj_id, brain_stg_states);
		vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, ventricle_ws_obj_id, ventricle_stg_states);

		// zoom scene
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ginfo.model_ws_obj_id, model_ws_states);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, brain_ws_obj_id, brain_ws_states);

		ventricle_ws_states.color[3] = 0.7;
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ventricle_ws_obj_id, ventricle_ws_states);

		// zoom stg scene
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ginfo.model_ws_obj_id, model_ws_states);
		brain_ws_states.is_wireframe = false;
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, brain_ws_obj_id, brain_ws_states);
		ventricle_ws_states.is_wireframe = false;
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ventricle_ws_obj_id, ventricle_ws_states);

		// model scene
		vzm::ObjStates model_ms_states, ventricle_ms_states;
		vzm::GetSceneObjectState(ginfo.model_scene_id, ginfo.model_ms_obj_id, model_ms_states);
		model_ms_states.color[3] = 0.1;
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, ginfo.model_ms_obj_id, model_ms_states);

		vzm::GetSceneObjectState(ginfo.model_scene_id, ventricle_ms_obj_id, ventricle_ms_states);
		ventricle_ms_states.color[0] = 1.0; ventricle_ms_states.color[1] = 0; ventricle_ms_states.color[2] = 0; ventricle_ms_states.color[3] = 0.7;
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, ventricle_ms_obj_id, ventricle_ms_states);
	}
	else {
		// object state setting
		vzm::ObjStates model_states, brain_states, ventricle_states;
		vzm::GetSceneObjectState(ginfo.model_scene_id, ginfo.model_ms_obj_id, model_states);

		brain_states = model_states;
		ventricle_states = model_states;

		model_states.color[3] = 0.7;
		brain_states.color[0] = 0.5; brain_states.color[1] = 0.5; brain_states.color[2] = 0.5; brain_states.color[3] = 0.3;
		ventricle_states.color[0] = 1.0; ventricle_states.color[1] = 0; ventricle_states.color[2] = 0; ventricle_states.color[3] = 1.0;

		// generate scene
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, ginfo.model_ms_obj_id, model_states);
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, brain_ms_obj_id, brain_states);
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, ventricle_ms_obj_id, ventricle_states);
	}
}
void UpdateTool(GlobalInfo& ginfo, track_info& trk_info, const std::string& probe_specifier_rb_name, int _probe_mode, Simulation& s)
{
	// (realsense, world, smartglass) scene
	var_settings::UpdateTrackInfo(&trk_info, probe_specifier_rb_name, _probe_mode);

	if (ginfo.is_modelaligned) {
		static int tool_ms_line_id = 0, tool_end_id = 0;

		glm::fvec3 sstool_p1_ws = ginfo.pos_probe_pin;
		glm::fvec3 sstool_p2_ws = ginfo.dir_probe_se * 0.2f;
		glm::fvec3 sstool_dir = ginfo.dir_probe_se;

		glm::fmat4 mat_os2ws = glm::inverse(ginfo.mat_ws2matchmodelfrm) * ginfo.mat_os2matchmodefrm;	// !!!
		// g_info.otrk_data.trk_info.GetLFrmInfo(name, mat_matchmodelfrm2ws);로 변경
		glm::fmat4 mat_ws2os = glm::inverse(mat_os2ws);
		glm::fvec3 sstool_p1_os = tr_pt(mat_ws2os, sstool_p1_ws);
		glm::fvec3 sstool_p2_os = tr_pt(mat_ws2os, sstool_p2_ws);

		// (model) ssu tool transformation //////////////////////////////////////////////////////////////
		int iToolIdx = -1;
		for (int i = 0, ni = s.rigidBodies.size(); i < ni; i++) {
			if (s.rigidBodies[i]->getType() == CiRigidBody::bodyType::TOOL) {
				iToolIdx = i;
				break;
			}
		}

		s.rigidBodies[iToolIdx]->m_visFiducialPoint[0] = btVector3(sstool_p1_os.x, sstool_p1_os.y, sstool_p1_os.z);
		s.rigidBodies[iToolIdx]->m_visFiducialPoint[1] = btVector3(sstool_p2_os.x, sstool_p2_os.y, sstool_p2_os.z);

		// (model) scene //////////////////////////////////////////////////////////////
		glm::fvec3 cyl_p03[2] = { sstool_p1_os, sstool_p2_os };
		float cyl_r = 1.5f;
		glm::fvec3 cyl_rgb = glm::fvec3(0, 1, 1);

		vzm::ObjStates tool_line_ms_state;
		double scale_factor = 0.001;
		glm::fmat4x4 mat_s = glm::scale(glm::fvec3(scale_factor));
		__cm4__ tool_line_ms_state.os2ws *= mat_s;

		vzm::GenerateCylindersObject((float*)cyl_p03, &cyl_r, __FP cyl_rgb, 1, tool_ms_line_id);
		vzm::ReplaceOrAddSceneObject(ginfo.model_scene_id, tool_ms_line_id, tool_line_ms_state);

		// (zoom, zoom stg) camera transformation, zoom scene //////////////////////////////////////////////////////////////
		// camera transformation
		vzm::CameraParameters zoom_cam_params;
		vzm::GetCameraParameters(zoom_scene_id, zoom_cam_params, zoom_cam_id);

		// camera transformation
		__cv3__ zoom_cam_params.pos = sstool_p1_ws + sstool_dir * 0.1f;
		__cv3__ zoom_cam_params.view = -sstool_dir;
		glm::fvec3 right = glm::normalize(glm::cross(-sstool_dir, glm::fvec3(0, 1, 0)));
		glm::fvec3 up = glm::normalize(glm::cross(right, -sstool_dir));
		__cv3__ zoom_cam_params.up = up;

		vzm::SetCameraParameters(zoom_scene_id, zoom_cam_params, zoom_cam_id);

		// sphere (zs)
		vzm::ObjStates model_states;
		model_states.color[3] = 0.3;

		vzm::GenerateSpheresObject(__FP glm::fvec4(sstool_p1_ws, 0.0015f), __FP glm::fvec3(0, 1, 1), 1, tool_end_id);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, tool_end_id, model_states);

		// zoom stg
		vzm::SetCameraParameters(zoom_scene_stg_id, zoom_cam_params, zoom_cam_id);
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, tool_end_id, model_states);
	}
}
void UpdateGuide(GlobalInfo& ginfo)
{
	
}
void UpdateZoomNavigation(GlobalInfo& ginfo)
{
	if (ginfo.is_modelaligned) {
		static int ssu_tool_guide_distance_id = 0, ssu_tool_guide_distance_text_id = 0;
		static int ssu_tool_guide_distance_arrow1_id = 0, ssu_tool_guide_distance_arrow2_id = 0;
		static int ssu_tool_guide_angleArrow_id = 0, ssu_tool_guide_angleText_id = 0;

		static int ssu_tool_guide_line_ws_id = 0, ssu_tool_guide_line_ms_id = 0;
		static int ssu_tool_guide_cylline_ws_id = 0;

		static int ssu_tool_guide_distanceLine_id = 0, ssu_tool_guide_distanceLineText_id = 0;
		static int ssu_tool_guide_angle_id = 0, ssu_tool_guide_angleText_id2 = 0;


		glm::fvec3 sstool_p1_ws = ginfo.pos_probe_pin;
		glm::fvec3 sstool_p2_ws = ginfo.dir_probe_se * 0.2f;
		glm::fvec3 sstool_dir = ginfo.dir_probe_se;

		glm::fvec3 ssguide_p1_ws = guide_lines[0];
		glm::fvec3 ssguide_p2_ws = guide_lines[1] * 0.2f;
		glm::fvec3 ssguide_dir = guide_lines[1];

		float fGuideAngle = glm::acos(glm::dot(sstool_dir, ssguide_dir)) * 180 / 3.141592;
		float fGuideDist = glm::distance(ssguide_p1_ws, sstool_p1_ws);

		glm::fvec3 tool_tip_ws = sstool_p1_ws;
		glm::fvec3 tool_dir_ws = sstool_dir;
		glm::fvec3 tool_right_ws = glm::normalize(glm::cross(tool_dir_ws, glm::fvec3(0, 1, 0)));
		glm::fvec3 tool_up_ws = glm::normalize(glm::cross(tool_right_ws, tool_dir_ws));
		glm::fvec3 tip2GuideEntry = ssguide_p2_ws - sstool_p1_ws;
		glm::fvec3 tip2GuideEnd = ssguide_p1_ws - sstool_p1_ws;
		glm::fvec3 guide_entry_ws = ssguide_p2_ws;
		glm::fvec3 guide_dir_ws = ssguide_dir;

		// draw direction line  ///////////////////////////////////////////////////////////////
		vzm::ObjStates distanceLineState, distanceArrowState;

		float dist_r = glm::dot(tip2GuideEntry, tool_right_ws);
		float dist_u = glm::dot(tip2GuideEntry, tool_up_ws);
		float dist_v = glm::dot(tip2GuideEnd, tool_dir_ws);

		std::vector<glm::fvec3> pos_lines(4), clr_lines(4);
		pos_lines[0] = tool_tip_ws; // r
		pos_lines[1] = dist_r * tool_right_ws + tool_tip_ws;
		pos_lines[2] = tool_tip_ws; // u
		pos_lines[3] = dist_u * tool_up_ws + tool_tip_ws;
		clr_lines[0] = clr_lines[1] = clr_lines[2] = clr_lines[3] = glm::fvec3(1.0, 1.0, 1.0);

		glm::fvec4 color = glm::fvec4(1, 0.5, 1, 0.5);
		__cm4__ distanceArrowState.os2ws = glm::fmat4(1.f);
		__cv4__ distanceArrowState.color = color;
		vzm::GenerateArrowObject((float*)&pos_lines[0], (float*)&pos_lines[1], 0.001f, ssu_tool_guide_distance_arrow1_id);
		vzm::GenerateArrowObject((float*)&pos_lines[0], (float*)&pos_lines[3], 0.001f, ssu_tool_guide_distance_arrow2_id);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ssu_tool_guide_distance_arrow1_id, distanceArrowState);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ssu_tool_guide_distance_arrow2_id, distanceArrowState);

		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ssu_tool_guide_distance_arrow1_id, distanceArrowState);
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ssu_tool_guide_distance_arrow2_id, distanceArrowState);

		string dist_str = std::to_string((int)(fGuideDist * 1000));
		auto MakeDistTextWidget = [&dist_str](const glm::fvec3 pos_lt, const vzm::CameraParameters& cam_param, const float size_font, int& text_id) {
			vector<glm::fvec3> text_xyzlt_view_up(3);
			text_xyzlt_view_up[0] = pos_lt;
			text_xyzlt_view_up[1] = __cv3__ cam_param.view;
			text_xyzlt_view_up[2] = __cv3__ cam_param.up;
			vzm::GenerateTextObject((float*)&text_xyzlt_view_up[0], dist_str, size_font, true, false, text_id);
		};

		float right_offset = -0.03f;
		vzm::CameraParameters zoom_cam_params;

		vzm::GetCameraParameters(zoom_scene_id, zoom_cam_params, zoom_cam_id);			// copy
		MakeDistTextWidget(tool_tip_ws + right_offset * tool_right_ws, zoom_cam_params, 0.01f, ssu_tool_guide_distance_text_id);

		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ssu_tool_guide_distance_id, distanceLineState);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ssu_tool_guide_distance_text_id, distanceLineState);

		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ssu_tool_guide_distance_id, distanceLineState);
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ssu_tool_guide_distance_text_id, distanceLineState);

		// draw angle(arrow, text) ///////////////////////////////////////////////////////////////
		vzm::ObjStates angleArrowState, angleTextState;

		string angle_str = std::to_string((int)fGuideAngle) + "˚";

		auto MakeAngleTextWidget = [&angle_str](const glm::fvec3 pos_lt, const vzm::CameraParameters& cam_param, const float size_font, int& text_id) {
			vector<glm::fvec3> text_xyzlt_view_up(3);
			text_xyzlt_view_up[0] = pos_lt;
			text_xyzlt_view_up[1] = __cv3__ cam_param.view;
			text_xyzlt_view_up[2] = __cv3__ cam_param.up;
			vzm::GenerateTextObject((float*)&text_xyzlt_view_up[0], angle_str, size_font, true, false, text_id);
		};

		// Text			
		right_offset = -0.02f;
		MakeAngleTextWidget(tool_tip_ws + right_offset * tool_right_ws, zoom_cam_params, 0.01f, ssu_tool_guide_angleText_id);
		vzm::ReplaceOrAddSceneObject(zoom_scene_id, ssu_tool_guide_angleText_id, angleTextState);
		vzm::ReplaceOrAddSceneObject(zoom_scene_stg_id, ssu_tool_guide_angleText_id, angleTextState);
	}
}
void UpdateSectionalImage(GlobalInfo& ginfo)
{
	if (ginfo.is_modelaligned) {
		vzm::SetRenderTestParam("_double3_3DTipPos", glm::dvec3(ginfo.pos_probe_pin), sizeof(glm::dvec3), -1, -1);

		glm::fvec3 dir_probe = glm::normalize(tr_vec(ginfo.mat_probe2ws, glm::fvec3(0, 0, 1)));
		var_settings::SetSectionalImageAssets(true, __FP ginfo.pos_probe_pin, __FP(ginfo.pos_probe_pin + dir_probe * 0.2f));
	}
	else {
		glm::fvec3 sstool_p1_ws = ginfo.pos_probe_pin;
		glm::fvec3 sstool_p2_ws = ginfo.dir_probe_se * 0.2f;
		glm::fvec3 sstool_dir = ginfo.dir_probe_se;

		glm::fvec3 ssguide_p1_ws = guide_lines[0];
		glm::fvec3 ssguide_p2_ws = guide_lines[1] * 0.2f;
		glm::fvec3 ssguide_dir = guide_lines[1];

		vzm::SetRenderTestParam("_double3_3DTipPos", glm::dvec3(sstool_p1_ws), sizeof(glm::dvec3), -1, -1);
		var_settings::SetSectionalImageAssets(true, __FP sstool_p1_ws, __FP(sstool_p1_ws + ssguide_dir * 0.2f));
	}
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

	rs_settings::InitializeRealsense(true, false, rs_w, rs_h, eye_w, eye_h);
	rs_settings::RunRsThread(original_data, filtered_data, eye_data);
	//rs2_intrinsics rgb_intrinsics, depth_intrinsics;
	//rs2_extrinsics rgb_extrinsics;
	//rs_settings::GetRsCamParams(rgb_intrinsics, depth_intrinsics, rgb_extrinsics);

	//////
	GlobalInfo* _ginfo;
	var_settings::GetVarInfoPtr((void**)&_ginfo);
	GlobalInfo& ginfo = *_ginfo;

	std::string pin_tool_name = "ss_tool_v2";

	InitializeVarSettings(ginfo);
	SetCvWindows(ginfo);
	SetPreoperations(ginfo, rs_w, rs_h, ws_w, ws_h, stg_w, stg_h, eye_w, eye_h);
	LoadPresets(ginfo, pin_tool_name);


	optitrk::SetRigidBodyEnabledbyName("ss_head", false);
	optitrk::SetRigidBodyEnabledbyName("marker", false);
	optitrk::SetRigidBodyEnabledbyName("breastbody", false);
	optitrk::SetRigidBodyEnabledbyName("spine", false);
	optitrk::SetRigidBodyEnabledbyName("tool_1", false);
	optitrk::SetRigidBodyEnabledbyName("tool_2", false);
	optitrk::SetRigidBodyEnabledbyName("tool_3", false);
	optitrk::SetRigidBodyEnabledbyName("ss_tool_v1", false);

	optitrk::SetRigidBodyEnabledbyName("rs_cam", true);
	optitrk::SetRigidBodyEnabledbyName("probe", true);
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
			static string _rb_names[NUM_RBS] = { "rs_cam" , "probe" , pin_tool_name, "ss_head" , "marker" };
			for (int i = 0; i < NUM_RBS; i++)
			{
				glm::fmat4x4 mat_lfrm2ws;
				bool is_detected = optitrk::GetRigidBodyLocationByName(_rb_names[i], (float*)&mat_lfrm2ws);
				cur_trk_info.SetLFrmInfo(_rb_names[i], is_detected, mat_lfrm2ws);
				if (_rb_names[i] == "ss_head")
					cur_trk_info.SetLFrmInfo(_rb_names[i], true, glm::fmat4x4());
			}

			optitrk::GetMarkersLocation(&cur_trk_info.mk_xyz_list, &cur_trk_info.mk_residue_list, &cur_trk_info.mk_cid_list);
			cur_trk_info.is_updated = true;
			track_que.push(cur_trk_info);
		}
	});

#ifdef EYE_VIS_RS
	string window_name_eye_view = "EYE VIEW";
	cv::namedWindow(ginfo.window_name_eye_view, WINDOW_NORMAL);
	cv::moveWindow(ginfo.window_name_eye_view, 2560, 0);
#endif


	// ssu ///////////////////////////////////////////////////////////////
	string modelRootPath("..\\Data");

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
			if (ginfo.is_modelaligned) {
				// Simulation
				QueryPerformanceCounter(&iT1);
				s.stepPhysics();

				QueryPerformanceCounter(&iT2);
				// 
				double dSimulationTime = (iT2.QuadPart - iT1.QuadPart) * 1000.0 / iTFrequency.QuadPart;
				double dt = dTimeStepMilliSecond - dSimulationTime;
				iT1 = iT2;

				s.accumulateTime(dSimulationTime);
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
	bool show_workload = true;
	bool is_ws_pick = false;

	auto DisplayTimes = [&show_workload](const LARGE_INTEGER lIntCntStart, const string& _test)
	{
		if (!show_workload) return;
		LARGE_INTEGER lIntFreq, lIntCntEnd;
		QueryPerformanceFrequency(&lIntFreq);
		QueryPerformanceCounter(&lIntCntEnd);
		double dRunTime1 = (lIntCntEnd.QuadPart - lIntCntStart.QuadPart) / (double)(lIntFreq.QuadPart);

		std::cout << _test << " : " << 1. / dRunTime1 << " fps" << endl;
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

	vzm::SetRenderTestParam("_double4_ShadingFactorsForGlobalPrimitives", glm::dvec4(0.8, 2.5, 1.0, 30.0), sizeof(glm::dvec4), 5, 1);

	vzm::SetRenderTestParam("_bool_GhostEffect", true, sizeof(bool), ginfo.rs_scene_id, 1);
	vzm::SetRenderTestParam("_bool_UseMask3DTip", true, sizeof(bool), -1, -1);
	vzm::SetRenderTestParam("_double4_MaskCenterRadius0", glm::dvec4(-100, -100, 0.07, 0.5), sizeof(glm::dvec4), -1, -1);
	vzm::SetRenderTestParam("_double3_HotspotParamsTKtKs0", glm::dvec3(0.0002, 0.5, 1.5), sizeof(glm::dvec3), -1, -1);
	vzm::SetRenderTestParam("_double_InDepthVis", 0.01, sizeof(double), -1, -1);
	vzm::SetRenderTestParam("_int_OitMode", (int)0, sizeof(int), -1, -1);
	//vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.stg_scene_id, 1, ginfo.model_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.stg_scene_id, 1, ginfo.brain_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.brain_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.ventricle_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.model_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.stg_scene_id, 1, ginfo.brain_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.stg_scene_id, 1, ginfo.ventricle_ws_obj_id);
	//vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.stg_scene_id, 1, ginfo.model_ws_obj_id);

	std::string probe_name = "probe";
	PROBE_MODE probe_mode = PROBE_MODE::DEFAULT;
	int line_guide_idx = 0;

	//optitrk::SetCameraSettings(0, 2, 50, 150);
	//optitrk::SetCameraSettings(1, 2, 50, 150);
	while (key_pressed != 'q' && key_pressed != 27)
	{
		LARGE_INTEGER frq_begin = GetPerformanceFreq();

		vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.model_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.brain_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsGhostSurface", true, sizeof(bool), ginfo.rs_scene_id, 1, brain_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.brain_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.ventricle_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ventricle_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, brain_ws_obj_id);
		vzm::SetRenderTestParam("_bool_IsOnlyHotSpotVisible", true, sizeof(bool), ginfo.rs_scene_id, 1, ginfo.model_ws_obj_id);

		bool load_calib_info = false;
		bool load_stg_calib_info = false;
		bool reset_calib = false;
		bool write_recoded_info = false;
		bool recompile_hlsl = false;
		switch (key_pressed) // http://www.asciitable.com/
		{
			case '[': postpone = max(postpone - 1, 0); std::cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
			case ']': postpone += 1; std::cout << "delay of IR tracker : " << postpone << "ms" << endl; break;
			case 'r': recompile_hlsl = true; std::cout << "Recompile Shader!" << endl; break;
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
			case 'o': vzm::SetRenderTestParam("_bool_UseSpinLock", false, sizeof(bool), -1, -1); break;
		}
		vzm::SetRenderTestParam("_bool_ReloadHLSLObjFiles", recompile_hlsl, sizeof(bool), -1, -1);
		vzm::SetRenderTestParam("_bool_PrintOutRoutineObjs", show_apis_console, sizeof(bool), -1, -1);
		//vzm::DisplayConsoleMessages(show_apis_console);

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
			//DisplayTimes(frq_begin, "device_stream_load");

			var_settings::UpdateTrackInfo(&trk_info);

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

			var_settings::SetTargetModelAssets("ss_head", __FP guide_lines[0], guide_lines.size() / 2, line_guide_idx);

			// SS tool custom vis.
			UpdateModel(ginfo, s, show_mks);							// Skin(head), Brain, Ventricle
			UpdateTool(ginfo, trk_info, probe_name, probe_mode, s);
			UpdateGuide(ginfo);
			UpdateZoomNavigation(ginfo);
			UpdateSectionalImage(ginfo);

			if (ginfo.is_modelaligned) {
				var_settings::RenderAndShowWindows(show_workload, image_rs_bgr, true);
				vzm::RenderScene(zoom_scene_id, zoom_cam_id);
				vzm::RenderScene(zoom_scene_stg_id, zoom_cam_stg_id);

				// get zoom buffer
				unsigned char *ptr_rgba_zv, *ptr_rgba_zv_stg;
				float *ptr_zdepth_zv, *ptr_zdepth_zv_stg;
				int w_zv, h_zv, w_zv_stg, h_zv_stg;
				bool bZoomBuffer = vzm::GetRenderBufferPtrs(zoom_scene_id, &ptr_rgba_zv, &ptr_zdepth_zv, &w_zv, &h_zv, zoom_cam_id);
				bool bZoomBuffer_stg = vzm::GetRenderBufferPtrs(zoom_scene_stg_id, &ptr_rgba_zv_stg, &ptr_zdepth_zv_stg, &w_zv_stg, &h_zv_stg, zoom_cam_stg_id);

				if (bZoomBuffer)
				{
					// realsense
					unsigned char* rs_buffer = image_rs_bgr.data;		// 3 channels bgr
					int nChan_zs = 4;
					int nChan_rs = 3;
					int nLeftTopX_rs = 30;
					int nLeftTopY_rs = 170;

					for (int y = 0; y < zoom_h; y++) {
						for (int x = 0; x < zoom_w; x++) {
							int idx_zv = nChan_zs * zoom_w*y + nChan_zs * x;
							int idx_rs = nChan_rs * rs_w*(y + nLeftTopY_rs) + nChan_rs * (x + nLeftTopX_rs);

							rs_buffer[idx_rs + 0] = ptr_rgba_zv[idx_zv + 0];
							rs_buffer[idx_rs + 1] = ptr_rgba_zv[idx_zv + 1];
							rs_buffer[idx_rs + 2] = ptr_rgba_zv[idx_zv + 2];
						}
					}
					imshow(ginfo.window_name_rs_view, image_rs_bgr);
				}

				if (bZoomBuffer_stg) {
					// smartglass
					int nChan_zs = 4;
					int nChan_stg = 4;
					int nLeftTopX_stg = 15;
					int nLeftTopY_stg = 165;

					int stg_cam_id = var_settings::GetCameraID_SSU(ginfo.stg_scene_id);
					float *ptr_zdepth_stg;
					int stg_w, stg_h;
					unsigned char* ptr_rgba_stg;
					bool bSmartGlassBuffer = vzm::GetRenderBufferPtrs(ginfo.stg_scene_id, &ptr_rgba_stg, &ptr_zdepth_stg, &stg_w, &stg_h, stg_cam_id);

					if (bSmartGlassBuffer) {
						for (int y = 0; y < zoom_stg_h; y++) {
							for (int x = 0; x < zoom_stg_w; x++) {
								int idx_zv_stg = nChan_zs * zoom_stg_w*y + nChan_zs * x;
								int idx_stg = nChan_stg * stg_w * (y + nLeftTopY_stg) + nChan_stg * (x + nLeftTopX_stg);

								ptr_rgba_stg[idx_stg + 0] = ptr_rgba_zv_stg[idx_zv_stg + 0];
								ptr_rgba_stg[idx_stg + 1] = ptr_rgba_zv_stg[idx_zv_stg + 1];
								ptr_rgba_stg[idx_stg + 2] = ptr_rgba_zv_stg[idx_zv_stg + 2];
								ptr_rgba_stg[idx_stg + 3] = ptr_rgba_zv_stg[idx_zv_stg + 3];
							}
						}

						Mat image_stg_bgr(cv::Size(stg_w, stg_h), CV_8UC4, (void*)ptr_rgba_stg);
						cv::rectangle(image_stg_bgr, cv::Rect(nLeftTopX_stg, nLeftTopY_stg, zoom_stg_w, zoom_stg_h), Scalar(255, 255, 255), 3);

						imshow(ginfo.window_name_stg_view, image_stg_bgr);
					}
				}
			}
			else {
				var_settings::RenderAndShowWindows(show_workload, image_rs_bgr, false);
			}
			cv::Mat img_rs_mirror(ginfo.rs_h, ginfo.rs_w, CV_8UC3, image_rs_bgr.data);
			imshow("rs mirror", img_rs_mirror);

			Show_Window(window_name_zs_view, zoom_scene_id, zoom_cam_id);

			int model_cam_id = var_settings::GetCameraID_SSU(ginfo.model_scene_id);
			Show_Window(ginfo.window_name_ms_view, ginfo.model_scene_id, model_cam_id);

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

	DeinitializeVarSettings(ginfo);

	// Signal threads to finish and wait until they do
	rs_settings::FinishRsThreads();
	rs_settings::DeinitializeRealsense();
	tracker_alive = false;
	tracker_processing_thread.join();

	optitrk::DeinitOptiTrackLib();
	vzm::DeinitEngineLib();

	// ssu ///////////////////////////////////////////////////////////////
	ssu_deform_alive = false;
	deform_processing_thread.join();

	s.destroySimulation();

	return 0;

}