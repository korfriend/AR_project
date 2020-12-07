#pragma once

#include <iostream>
#include <windows.h>

#include <queue>
#include <bitset>
#include <string>
#include <sstream>
#include <iomanip>


#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/transform.hpp>
#include <glm/gtc/constants.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/vector_angle.hpp>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/video/tracking.hpp>

#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)
#define __PR(A, INTERVAL) A[0] << INTERVAL << A[1] << INTERVAL << A[2]

using namespace std;
using namespace cv;

glm::fvec3 tr_pt(const glm::fmat4x4& mat, const glm::fvec3& p)
{
	glm::fvec4 _p(p, 1.f);
	_p = mat * _p;
	return glm::fvec3(_p.x / _p.w, _p.y / _p.w, _p.z / _p.w);
}

glm::fvec3 tr_vec(const glm::fmat4x4& mat, const glm::fvec3& v)
{
	glm::fmat3x3 r = mat;
	glm::fvec3 _v = r * v;
	return _v;
}

// http://stackoverflow.com/questions/236129/how-to-split-a-string-in-c
std::vector<std::string> split(const std::string &text, char sep) {
	std::vector<std::string> tokens;
	int start = 0, end = 0;
	while ((end = text.find(sep, start)) != std::string::npos) {
		tokens.push_back(text.substr(start, end - start));
		start = end + 1;
	}
	tokens.push_back(text.substr(start));
	return tokens;
}
// https://stackoverflow.com/questions/9150538/how-do-i-tostring-an-enum-in-c
#define ENUM(name, ...)\
enum name \
{\
__VA_ARGS__\
};\
std::vector<std::string> name##Map = split(#__VA_ARGS__, ',');\
    std::string EtoString(const name v) { return name##Map.at(v);}


#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
// https://www.justsoftwaresolutions.co.uk/threading/implementing-a-thread-safe-queue-using-condition-variables.html
template<typename Data>
class concurrent_queue // single consumer
{
private:
	std::queue<Data> the_queue;
	mutable boost::mutex the_mutex;

	boost::condition_variable the_condition_variable;
	int MAX_QUEUE;

public:
	concurrent_queue(int cap = 10)
	{
		MAX_QUEUE = cap;
	}

	void wait_for_data()
	{
		boost::mutex::scoped_lock lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}
	}

	void wait_and_pop(Data& popped_value)
	{
		boost::mutex::scoped_lock lock(the_mutex);
		while (the_queue.empty())
		{
			the_condition_variable.wait(lock);
		}

		popped_value = the_queue.front();
		the_queue.pop();
	}

	//void postphone_back_and_clear(Data& popped_value)
	//{
	//	boost::mutex::scoped_lock lock(the_mutex);
	//	while (the_queue.size() < MAX_QUEUE)
	//	{
	//		the_condition_variable.wait(lock);
	//	}
	//
	//	popped_value = the_queue.back();
	//	std::queue<Data> empty_queue;
	//	std::swap(the_queue, empty_queue);
	//}

	void push(const Data& data) //enque
	{
		//boost::mutex::scoped_lock lock(the_mutex);
		//the_queue.push(data);

		boost::mutex::scoped_lock lock(the_mutex);
		bool const was_empty = the_queue.empty();
		the_queue.push(data);
		if (the_queue.size() > MAX_QUEUE)
		{
			the_queue.pop();
		}

		lock.unlock(); // unlock the mutex

		if (was_empty)
		{
			the_condition_variable.notify_one();
		}
	}

	bool empty() const
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.empty();
	}

	Data& front()
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.front();
	}

	Data const& front() const
	{
		boost::mutex::scoped_lock lock(the_mutex);
		return the_queue.front();
	}

	void pop() // deque
	{
		boost::mutex::scoped_lock lock(the_mutex);
		the_queue.pop();
	}
};

struct track_info
{
	//  "rs_cam" , "probe" , "ss_tool_v1" , "ss_head" , "breastbody" 
	map<string, pair<bool, glm::fmat4x4>> map_lfrm2ws;

	bool GetLFrmInfo(const string& name, glm::fmat4x4& mat_lfrm2ws) const
	{
		bool is_detected = false;
		auto it = map_lfrm2ws.find(name);
		if (it != map_lfrm2ws.end())
		{
			is_detected = get<0>(it->second);
			mat_lfrm2ws = get<1>(it->second);
		}
		return is_detected;
	}

	void SetLFrmInfo(const string& name, const bool is_detected, const glm::fmat4x4& mat_lfrm2ws)
	{
		map_lfrm2ws[name] = pair<bool, glm::fmat4x4>(is_detected, mat_lfrm2ws);
	}
	// when changing this, please consider the serialization
	//"rs_cam"
	//glm::fmat4x4 mat_rbcam2ws;
	//bool is_detected_rscam;
	//
	////"probe"
	//glm::fmat4x4 mat_probe2ws;
	//bool is_detected_probe;
	//
	////"ss_tool_v1"
	//glm::fmat4x4 mat_tfrm2ws;
	//bool is_detected_sstool;
	//
	////"ss_head"
	//glm::fmat4x4 mat_headfrm2ws;
	//bool is_detected_sshead;
	//
	////"breastbody"
	//glm::fmat4x4 mat_bodyfrm2ws;
	//bool is_detected_brbody;

	std::vector<float> mk_xyz_list;
	std::vector<float> mk_residue_list;
	std::vector<std::bitset<128>> mk_cid_list;

	bool is_updated;
	track_info() { is_updated = false; }

	bool GetProbePinPoint(glm::fvec3& pos)
	{
		glm::fmat4x4 mat_lfrm2ws;
		if (GetLFrmInfo("probe", mat_lfrm2ws))
		{
			pos = tr_pt(mat_lfrm2ws, glm::fvec3(0));
			return true;
		}
		return false;
	}

	glm::fvec3 GetMkPos(int idx)
	{
		if (idx < 0 || idx >= mk_xyz_list.size() / 3) return glm::fvec3(0);
		return ((glm::fvec3*)&mk_xyz_list[0])[idx];
	}

	char* GetSerialBuffer(size_t& bytes_size)
	{
		int num_mks = (int)mk_xyz_list.size();
		int num_lfrms = (int)map_lfrm2ws.size();
		bytes_size = (sizeof(glm::fmat4x4) + sizeof(bool) + sizeof(char) * 100) * num_lfrms + (sizeof(float) * 2 + (128 / 8)) * num_mks + 4 * 2; // last 4 * 2 means num_mks, num_lfrms
		char* buf = new char[bytes_size];
		memset(buf, 0, bytes_size);
		*(int*)&buf[0] = num_mks;
		*(int*)&buf[4] = num_lfrms;

		memcpy(&buf[8], &mk_xyz_list[0], sizeof(float) * num_mks);
		memcpy(&buf[8 + sizeof(float) * num_mks], &mk_residue_list[0], sizeof(float) * num_mks);
		memcpy(&buf[8 + sizeof(float) * num_mks * 2], &mk_cid_list[0], (128 / 8) * num_mks);

		int offset = 8 + sizeof(float) * num_mks * 2 + (128 / 8) * num_mks;
		int unit_size = sizeof(bool) + sizeof(glm::fmat4x4) + sizeof(char) * 100;
		int i = 0;
		for (auto it = map_lfrm2ws.begin(); it != map_lfrm2ws.end(); it++, i++)
		{
			memcpy(&buf[offset + unit_size * i], it->first.c_str(), sizeof(char) * it->first.length());
			*(bool*)&buf[offset + unit_size * i + 100] = get<0>(it->second);
			*(glm::fmat4x4*)&buf[offset + unit_size * i + 100 + sizeof(bool)] = get<1>(it->second);
		}

		return buf;
	}

	void SetFromSerialBuffer(const char* buf)
	{
		int num_mks = *(int*)buf[0];
		int num_lfrms = *(int*)buf[4];
		mk_xyz_list.assign(num_mks, 0);
		mk_residue_list.assign(num_mks, 0);
		mk_cid_list.assign(num_mks, 0);

		memcpy(&mk_xyz_list[0], &buf[8], sizeof(float) * num_mks);
		memcpy(&mk_residue_list[0], &buf[8 + sizeof(float) * num_mks], sizeof(float) * num_mks);
		memcpy(&mk_cid_list[0], &buf[8 + sizeof(float) * num_mks * 2], (128 / 8) * num_mks);

		int offset = 8 + sizeof(float) * num_mks * 2 + (128 / 8) * num_mks;
		int unit_size = sizeof(bool) + sizeof(glm::fmat4x4) + sizeof(char) * 100;
		for (int i = 0; i < num_lfrms; i++)
		{
			char arry[100];
			memcpy(arry, &buf[offset + unit_size * i], sizeof(char) * 100);
			string name = arry;
			bool is_detected = *(bool*)&buf[offset + unit_size * i + 100];
			glm::fmat4x4 mat_lfrm2ws = *(glm::fmat4x4*)&buf[offset + unit_size * i + 100 + sizeof(bool)];
			SetLFrmInfo(name, is_detected, mat_lfrm2ws);
		}
	}

	bool CheckExistCID(const std::bitset<128>& cid, int* mk_idx = NULL)
	{
		bool exist_mk_cid = false;
		if (mk_idx) *mk_idx = -1;
		for (int i = 0; i < (int)mk_cid_list.size(); i++)
		{
			if (mk_cid_list[i] == cid)
			{
				exist_mk_cid = true;
				if (mk_idx) *mk_idx = i;
				break;
			}
		}

		return exist_mk_cid;
	}
};

struct OpttrkData
{
	track_info trk_info; // available when USE_OPTITRACK
	vzm::ObjStates obj_state;
	vector<int> armk_text_ids;
	int rs_lf_axis_id, probe_lf_axis_id; // lf means local frame
	vector<pair<Point2f, Point3f>> tc_calib_pt_pairs;
	vector<pair<Point2f, Point3f>> stg_calib_pt_pairs;
	vector<pair<Point2f, Point3f>> stg_calib_pt_pairs_2;
	bitset<128> stg_calib_mk_cid;
	vector<int> calib_trial_rs_cam_frame_ids; // deprecated!!
	vector<int> mk_pickable_sphere_ids;

	map<string, vector<Point3f>> custom_pos_map;
	string marker_rb_name;
	vector<Point3f> calib_3d_pts;

	OpttrkData()
	{
		marker_rb_name = "";
		stg_calib_mk_cid = 0;
		rs_lf_axis_id = probe_lf_axis_id = 0;
		obj_state.emission = 0.4f;
		obj_state.diffusion = 0.6f;
		obj_state.specular = 0.2f;
		obj_state.sp_pow = 30.f;
		__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
		__cm4__ obj_state.os2ws = glm::fmat4x4();
	}
};

ENUM(RsTouchMode, None, Pick, AR_Marker, DST_TOOL_E0, DST_TOOL_SE0, DST_TOOL_SE1, FIX_SCREW, Calib_TC, PIN_ORIENTATION, Calib_STG, Calib_STG2, Align, ICP, Capture, Pair_Clear, STG_Pair_Clear)

// added by dojo at 200813
struct SS_Tool_Guide_Pts
{
	int ss_tool_guide_points_id;
	vector<glm::fvec3> pos_centers_tfrm;
	SS_Tool_Guide_Pts() { ss_tool_guide_points_id = 0; }
};

struct ButtonState
{
	RsTouchMode mode;
	Rect rect;
	int touch_count;
	bool is_activated;
	bool is_subbutton;
	std::string name;
	cv::Scalar activated_bg;

	ButtonState()
	{
		touch_count = 0;
		is_activated = false;
		is_subbutton = false;
	}

	ButtonState(const RsTouchMode _mode, const Rect& _rect, const int _count, const bool _is_activated, const std::string& _name, const bool _is_subbutton, const cv::Scalar& _activated_bg)
	{
		mode = _mode;
		rect = _rect;
		touch_count = 0;
		is_activated = _is_activated;
		name = _name;
		activated_bg = _activated_bg;
		is_subbutton = _is_subbutton;
	}
};

struct GlobalInfo
{
	int scenario;

	map<int, glm::fvec3> vzmobjid2pos;

	RsTouchMode touch_mode;
	bool skip_call_render;
	OpttrkData otrk_data;
	bool is_calib_rs_cam;
	bool is_calib_stg_cam;
	bool is_calib_stg_cam_2;
	int stg_display_num;

	bool is_probe_detected;
	glm::fvec3 pos_probe_pin;
	glm::fvec3 dir_probe_se;
	glm::fmat4x4 mat_probe2ws;
	string src_tool_name;
	string dst_tool_name;

	// guide lines w.r.t. target asset coord.
	vector<std::pair<glm::fvec3, glm::fvec3>> guide_lines_target_rbs;
	
	// model related
	bool is_modelaligned;
	int model_ms_obj_id;
	int model_ws_obj_id;
	int model_volume_id; // only for ws
	glm::fmat4x4 mat_ws2matchmodelfrm;
	glm::fmat4x4 mat_os2matchmodefrm;
	glm::fmat4x4 mat_matchtr;
	int captured_model_ms_point_id;
	int captured_model_ws_point_id;
	int rs_pc_id;

	vector<glm::fvec3> model_ms_pick_pts;
	vector<glm::fvec3> model_rbs_pick_pts;
	std::string match_model_rbs_name;
	int model_ws_pick_spheres_id;
	int model_ms_pick_spheres_id;

	SS_Tool_Guide_Pts ss_tool_info;
	vector<glm::fvec3> tool_guide_pos_os;
	int brain_ms_obj_id, ventricle_ms_obj_id;
	int brain_ws_obj_id, ventricle_ws_obj_id;

	int rs_w, rs_h;
	int stg_w, stg_h;
	int eye_w, eye_h;
	int ws_w, ws_h;
	int ms_w, ms_h;

	// scene definition
	int ws_scene_id; // arbitrary integer
	int rs_scene_id; // arbitrary integer
	int model_scene_id; // arbitrary integer
	int csection_scene_id; // arbitrary integer
	int stg_scene_id; // arbitrary integer
	int zoom_scene_id;

	// cv window name
	string window_name_rs_view;
	string window_name_ws_view;
	string window_name_ms_view;
	string window_name_stg_view;
	string window_name_zs_view;

	// file path
	string optrack_calib;
	string optrack_env;
	string cb_positions;
	string rs_calib;
	string stg_calib;
	string sst_positions;
	string model_path;
	string volume_model_path;
	string model_predefined_pts;
	string guide_path;		// 20200818 add guide path name
	string model_view_preset;

	map<string, string> custom_pos_file_paths;

	// rs cam ui buttons
	map<RsTouchMode, ButtonState> rs_buttons;

	GlobalInfo()
	{
		touch_mode = RsTouchMode::None;
		skip_call_render = false;
		is_calib_rs_cam = false;
		is_calib_stg_cam = false;
		is_calib_stg_cam_2 = false;
		model_ms_obj_id = 0;
		model_ws_obj_id = 0;
		captured_model_ms_point_id = 0;
		is_modelaligned = false;
		rs_pc_id = 0;
		model_volume_id = 0;
		is_probe_detected = false;
		stg_display_num = 1;

		// SSU
		brain_ms_obj_id = 0;
		brain_ws_obj_id = 0;
		ventricle_ms_obj_id = 0;
		ventricle_ws_obj_id = 0;

		scenario = 0;
	}
};

enum PROBE_MODE
{
	DEFAULT = 0, // PIN_POS AND DIR
	ONLY_PIN_POS,
	ONLY_RBFRAME,
};

#define TESTOUT(NAME, P) {cout << NAME << P.x << ", " << P.y << ", " << P.z << endl;}
void ComputeDXProjectionMatrix(float* mat_cs2ps, float* mat_ps2ss, const float* mat_int3x3,
	const float x0, const float y0, const float width, const float height, const float znear, const float zfar)
{
	float q = zfar / (znear - zfar);
	float qn = zfar * znear / (znear - zfar);
	//float depth = zfar - znear;
	//float q = -(zfar + znear) / depth;
	//float qn = -2.0f * (zfar * znear) / depth;
	// This follows the OpenGL convention where positive Y coordinates goes down

	glm::fmat4x4 mat_p;
	glm::fmat3x3 K = *(glm::mat3x3*)mat_int3x3;

	float fx = K[0][0];
	float fy = K[1][1];
	float sc = K[1][0];
	float cx = K[2][0];
	float cy = K[2][1];

	mat_p[0][0] = 2.f*fx / width;
	mat_p[1][0] = -2.f*sc / width;
	mat_p[2][0] = (width + 2.f * x0 - 2.f*cx) / width;
	mat_p[3][0] = 0;
	mat_p[0][1] = 0;
	mat_p[1][1] = 2.f*fy / height;
	mat_p[2][1] = -(height + 2.f * y0 - 2.f*cy) / height;
	mat_p[3][1] = 0;
	mat_p[0][2] = 0;
	mat_p[1][2] = 0;
	mat_p[2][2] = q;
	mat_p[3][2] = qn;
	mat_p[0][3] = 0;
	mat_p[1][3] = 0;
	mat_p[2][3] = -1.f;
	mat_p[3][3] = 0;

	__cm4__ mat_cs2ps = mat_p;

	glm::fmat4x4 matTranslate, matScale, matTransform, matTranslateSampleModel;
	matTranslate = glm::translate(glm::fvec3(1., -1., 0.));
	matScale = glm::scale(glm::fvec3(width*0.5, height*0.5, 1.));
	//matTranslateSampleModel = glm::translate(glm::fvec3(-0.5, 0.5, 0.));

	glm::fmat4x4 mat_s;
	mat_s = matTranslateSampleModel * (matScale * matTranslate);

	mat_s[0][1] *= -1.;
	mat_s[1][1] *= -1.;
	mat_s[2][1] *= -1.;
	mat_s[3][1] *= -1.;

	__cm4__ mat_ps2ss = mat_s;
}

void Make_Viewport(std::vector<glm::fvec3>& pos_tris, std::vector<glm::fvec3>& clr_tris, std::vector<glm::fvec3>& pos_lines, std::vector<glm::fvec3>& clr_lines,
	const glm::fvec3 clr_tri, const glm::fvec3 clr_line,
	const glm::fvec3 pos_c, const glm::fvec3 view, const glm::fvec3 up, const float fov_hor, const float fov_ver, const float max_length)
{
	glm::fvec3 right = glm::cross(view, up); // x

	glm::fvec3 pos_e = pos_c + view * max_length;
	// up
	float yhalf = tan(fov_ver / 2.f) * max_length;
	float xhalf = tan(fov_hor / 2.f) * max_length;
	glm::fvec3 vec_y = up * yhalf;
	glm::fvec3 vec_x = right * yhalf;

	glm::fvec3 pos_0 = pos_c;
	glm::fvec3 pos_1 = pos_e + vec_x + vec_y;
	glm::fvec3 pos_2 = pos_e - vec_x + vec_y;
	glm::fvec3 pos_3 = pos_e + vec_x - vec_y;
	glm::fvec3 pos_4 = pos_e - vec_x - vec_y;

	pos_tris.push_back(pos_0);
	pos_tris.push_back(pos_3);
	pos_tris.push_back(pos_1);

	pos_tris.push_back(pos_0);
	pos_tris.push_back(pos_1);
	pos_tris.push_back(pos_2);

	pos_tris.push_back(pos_0);
	pos_tris.push_back(pos_2);
	pos_tris.push_back(pos_4);

	pos_tris.push_back(pos_0);
	pos_tris.push_back(pos_4);
	pos_tris.push_back(pos_3);

	pos_tris.push_back(pos_1);
	pos_tris.push_back(pos_3);
	pos_tris.push_back(pos_2);
	pos_tris.push_back(pos_2);
	pos_tris.push_back(pos_3);
	pos_tris.push_back(pos_4);

	pos_lines.push_back(pos_0);
	pos_lines.push_back(pos_1);
	pos_lines.push_back(pos_0);
	pos_lines.push_back(pos_2);
	pos_lines.push_back(pos_0);
	pos_lines.push_back(pos_3);
	pos_lines.push_back(pos_0);
	pos_lines.push_back(pos_4);

	pos_lines.push_back(pos_1);
	pos_lines.push_back(pos_2);
	pos_lines.push_back(pos_2);
	pos_lines.push_back(pos_4);
	pos_lines.push_back(pos_4);
	pos_lines.push_back(pos_3);
	pos_lines.push_back(pos_3);
	pos_lines.push_back(pos_1);

	for (int i = 0; i < pos_tris.size(); i++)
		clr_tris.push_back(clr_tri);
	for (int i = 0; i < pos_lines.size(); i++)
		clr_lines.push_back(clr_line);
}

void Cam_Gen(const glm::vec3& pos_cam_ws, const glm::vec3& view_ws, const glm::vec3& up_ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
{
	std::vector<glm::fvec3> pos_tris, pos_lines, clr_tris, clr_lines;
	Make_Viewport(pos_tris, clr_tris, pos_lines, clr_lines, glm::fvec3(1, 0.3, 0.5), glm::fvec3(1, 0.3, 0.5),
		pos_cam_ws, view_ws, up_ws, 3.141592654f * 56.f / 180.f, 3.141592654f * 46.f / 180.f, 0.1f);
	vzm::GenerateTrianglesObject((float*)&pos_tris[0], (float*)&clr_tris[0], (int)pos_tris.size() / 3, cam_tris_is);
	vzm::GenerateLinesObject((float*)&pos_lines[0], (float*)&clr_lines[0], (int)pos_lines.size() / 2, cam_lines_id);

	std::vector<glm::fvec3> pinfo(3);
	pinfo[0] = pos_lines[5];
	pinfo[1] = -view_ws;
	pinfo[2] = up_ws;
	vzm::GenerateTextObject((float*)&pinfo[0], cam_label, 0.05, true, false, cam_label_id);
}

void Cam_Gen(const glm::fmat4x4& mat_cs2ws, const string cam_label, int& cam_tris_is, int& cam_lines_id, int& cam_label_id)
{
	glm::fvec3 pos_cam_ws = tr_pt(mat_cs2ws, glm::fvec3());
	glm::fvec3 view_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 0, -1));
	glm::fvec3 up_ws = tr_vec(mat_cs2ws, glm::fvec3(0, 1, 0));
	Cam_Gen(pos_cam_ws, view_ws, up_ws, cam_label, cam_tris_is, cam_lines_id, cam_label_id);
}

void Update_CamModel(const int scene_id, const glm::fmat4x4& mat_frm2ws, const string cam_label, const int cam_id)
{
	static int obj_ids_buf[300] = {};
	glm::fvec3 pos_c = tr_pt(mat_frm2ws, glm::fvec3(0, 0, 0));
	glm::fvec3 up = tr_vec(mat_frm2ws, glm::fvec3(0, 1, 0));
	glm::fvec3 view = tr_vec(mat_frm2ws, glm::fvec3(0, 0, -1));
	Cam_Gen(pos_c, view, up, cam_label, obj_ids_buf[3 * cam_id], obj_ids_buf[3 * cam_id + 1], obj_ids_buf[3 * cam_id + 2]);
	
	vzm::ObjStates obj_state;
	obj_state.emission = 0.4f;
	obj_state.diffusion = 0.6f;
	obj_state.specular = 0.2f;
	obj_state.sp_pow = 30.f;
	__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
	__cm4__ obj_state.os2ws = glm::fmat4x4();
	
	vzm::ReplaceOrAddSceneObject(scene_id, obj_ids_buf[3 * cam_id + 0], obj_state);

	vzm::ObjStates thickline_state = obj_state;
	thickline_state.line_thickness = 3;
	vzm::ReplaceOrAddSceneObject(scene_id, obj_ids_buf[3 * cam_id + 1], thickline_state);
	vzm::ReplaceOrAddSceneObject(scene_id, obj_ids_buf[3 * cam_id + 2], obj_state);
}

void Axis_Gen(const glm::fmat4x4& mat_frame2ws, const float axis_line_leng, int& axis_obj_id)
{
	glm::fvec3 pos_c = tr_pt(mat_frame2ws, glm::fvec3(0));
	glm::fvec3 vec_x = glm::normalize(tr_vec(mat_frame2ws, glm::fvec3(1, 0, 0))) * axis_line_leng;
	glm::fvec3 vec_y = glm::normalize(tr_vec(mat_frame2ws, glm::fvec3(0, 1, 0))) * axis_line_leng;
	glm::fvec3 vec_z = glm::normalize(tr_vec(mat_frame2ws, glm::fvec3(0, 0, 1))) * axis_line_leng;

	std::vector<glm::fvec3> xyz_axis_lines;
	std::vector<glm::fvec3> clr_axis_lines;
	std::vector<float> r_axis_lines;
	xyz_axis_lines.push_back(pos_c);
	xyz_axis_lines.push_back(pos_c + vec_x);
	xyz_axis_lines.push_back(pos_c);
	xyz_axis_lines.push_back(pos_c + vec_y);
	xyz_axis_lines.push_back(pos_c);
	xyz_axis_lines.push_back(pos_c + vec_z);
	clr_axis_lines.push_back(glm::fvec3(1, 0, 0));
	clr_axis_lines.push_back(glm::fvec3(0, 1, 0));
	clr_axis_lines.push_back(glm::fvec3(0, 0, 1));
	float axis_r = axis_line_leng * 0.07f;
	r_axis_lines.push_back(axis_r);
	r_axis_lines.push_back(axis_r);
	r_axis_lines.push_back(axis_r);
	r_axis_lines.push_back(axis_r);
	//vzm::GenerateLinesObject((float*)&xyz_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_lines_obj_id);
	vzm::GenerateCylindersObject((float*)&xyz_axis_lines[0], (float*)&r_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_obj_id);

}

void World_GridAxis_Gen(int& coord_grid_obj_id, int& axis_lines_obj_id, int& axis_texX_obj_id, int& axis_texZ_obj_id)
// make world coordinate grid objects
{
	std::vector<glm::fvec3> xyz_coord_grid;
	std::vector<glm::fvec3> clr_coord_grid;
	std::vector<float> r_coord_grid;
	int count_cyls = 0;
	double cyl_interval = 0.1;
	for (int i = 0; i < 200; i++)
	{
		xyz_coord_grid.push_back(glm::fvec3(-10, 0, -10 + i * cyl_interval));
		xyz_coord_grid.push_back(glm::fvec3(10, 0, -10 + i * cyl_interval));

		xyz_coord_grid.push_back(glm::fvec3(-10 + i * cyl_interval, 0, -10));
		xyz_coord_grid.push_back(glm::fvec3(-10 + i * cyl_interval, 0, 10));

		glm::fvec3 _color(0.3, 0.3, 0.3);
		float _r = 0.002f;
		if (i % 5 == 0)
		{
			_color = glm::fvec3(0.8, 0.8, 0.8);
			_r = 0.003f;
		}
		clr_coord_grid.push_back(_color);
		clr_coord_grid.push_back(_color);
		r_coord_grid.push_back(_r);
		clr_coord_grid.push_back(_color);
		clr_coord_grid.push_back(_color);
		r_coord_grid.push_back(_r);
	}
	//vzm::GenerateCylindersObject((float*)&pos_cyl_coord_plane[0], (float*)&r_cyl_coord_plane[0], (float*)&clr_cyl_coord_plane[0], (int)r_cyl_coord_plane.size(), coord_plane_obj2);
	vzm::GenerateLinesObject((float*)&xyz_coord_grid[0], (float*)&clr_coord_grid[0], (int)xyz_coord_grid.size() / 2, coord_grid_obj_id);

	std::vector<glm::fvec3> xyz_axis_lines;
	std::vector<glm::fvec3> clr_axis_lines;
	std::vector<float> r_axis_lines;
	xyz_axis_lines.push_back(glm::fvec3());
	xyz_axis_lines.push_back(glm::fvec3(0.1, 0, 0));
	xyz_axis_lines.push_back(glm::fvec3());
	xyz_axis_lines.push_back(glm::fvec3(0, 0.1, 0));
	xyz_axis_lines.push_back(glm::fvec3());
	xyz_axis_lines.push_back(glm::fvec3(0, 0, 0.1));
	clr_axis_lines.push_back(glm::fvec3(1, 0, 0));
	clr_axis_lines.push_back(glm::fvec3(0, 1, 0));
	clr_axis_lines.push_back(glm::fvec3(0, 0, 1));
	r_axis_lines.push_back(0.007f);
	r_axis_lines.push_back(0.007f);
	r_axis_lines.push_back(0.007f);
	r_axis_lines.push_back(0.007f);
	//vzm::GenerateLinesObject((float*)&xyz_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_lines_obj_id);
	vzm::GenerateCylindersObject((float*)&xyz_axis_lines[0], (float*)&r_axis_lines[0], (float*)&clr_axis_lines[0], (int)clr_axis_lines.size(), axis_lines_obj_id);

	std::vector<glm::fvec3> pinfo(3);
	pinfo[0] = glm::fvec3(0.10, 0, 0);
	pinfo[1] = glm::fvec3(0, -1, 0);
	pinfo[2] = glm::fvec3(1, 0, 0);
	vzm::GenerateTextObject((float*)&pinfo[0], "X", 0.07, true, false, axis_texX_obj_id);
	pinfo[0] = glm::fvec3(0, 0, 0.10);
	pinfo[1] = glm::fvec3(0, -1, 0);
	pinfo[2] = glm::fvec3(0, 0, 1);
	vzm::GenerateTextObject((float*)&pinfo[0], "Z", 0.07, true, false, axis_texZ_obj_id);
}

void GenWorldGrid(int ws_scene_id, int ws_cam_id)
{
	int coord_grid_obj_id = 0, axis_lines_obj_id = 0, axis_texX_obj_id = 0, axis_texZ_obj_id = 0;
	World_GridAxis_Gen(coord_grid_obj_id, axis_lines_obj_id, axis_texX_obj_id, axis_texZ_obj_id);
	vzm::ObjStates grid_obj_state;
	grid_obj_state.color[3] = 0.7f;
	grid_obj_state.line_thickness = 0;
	vzm::ReplaceOrAddSceneObject(ws_scene_id, coord_grid_obj_id, grid_obj_state);
	bool foremost_surf_rendering = false;
	vzm::SetRenderTestParam("_bool_OnlyForemostSurfaces", foremost_surf_rendering, sizeof(bool), ws_scene_id, ws_cam_id, coord_grid_obj_id);
	grid_obj_state.color[3] = 0.9f;
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_lines_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(1, 0.3, 0.3, 0.6);
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_texX_obj_id, grid_obj_state);
	*(glm::fvec4*) grid_obj_state.color = glm::fvec4(0.3, 0.3, 1, 0.6);
	vzm::ReplaceOrAddSceneObject(ws_scene_id, axis_texZ_obj_id, grid_obj_state);
}

void ComputeClosestPointBetweenLineAndPoint(const glm::fvec3& pos_line, const glm::fvec3& dir_line, const glm::fvec3& pos_point, glm::fvec3& pos_closest_point)
{
	float len = glm::length(dir_line);
	if (len <= 0.000001f) return;
	//http://math.stackexchange.com/questions/748315/finding-the-coordinates-of-a-point-on-a-line-that-produces-the-shortest-distance
	float t = ((pos_point.x * dir_line.x + pos_point.y * dir_line.y + pos_point.z * dir_line.z) - (pos_line.x * dir_line.x + pos_line.y * dir_line.y + pos_line.z * dir_line.z)) / len;
	pos_closest_point = pos_line + dir_line * t;
}

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
	std::ostringstream out;
	out << std::setprecision(n) << a_value;
	return out.str();
}

void SetDashEffectInRendering(const int scene_id, const int cam_id, const int line_obj_id, const double dash_interval, const bool is_InvertColorDashLine)
{
	vzm::SetRenderTestParam("_bool_IsDashed", true, sizeof(bool), scene_id, cam_id, line_obj_id);
	vzm::SetRenderTestParam("_bool_IsInvertColorDashLine", is_InvertColorDashLine, sizeof(bool), scene_id, cam_id, line_obj_id);
	vzm::SetRenderTestParam("_double_LineDashInterval", dash_interval, sizeof(double), scene_id, cam_id, line_obj_id);
}

void MakeDistanceLine(const int scene_id, const glm::fvec3& pos_tool_tip, const glm::fvec3& pos_dst_point, const float font_size, int& closest_point_line_id, int& dist_text_id)
{
	//const float font_size = 30.f; // mm
	//static int closest_point_line_id = 0, dist_text_id = 0;
	glm::fvec3 pos_closest_line[2] = { pos_tool_tip, pos_dst_point };
	glm::fvec3 clr_closest_line[2] = { glm::fvec3(1, 1, 0), glm::fvec3(1, 1, 0) };
	vzm::GenerateLinesObject(__FP pos_closest_line, __FP clr_closest_line, 1, closest_point_line_id);
	//vzm::ObjStates obj_state_closest_point_line;
	//obj_state_closest_point_line.line_thickness = 2;
	//vzm::ReplaceOrAddSceneObject(0, closest_point_line_id, obj_state_closest_point_line);
	//vzm::SetRenderTestParam("_bool_IsDashed", true, sizeof(bool), 0, 0, closest_point_line_id);
	//vzm::SetRenderTestParam("_bool_IsInvertColorDashLine", true, sizeof(bool), 0, 0, closest_point_line_id);
	//vzm::SetRenderTestParam("_double_LineDashInterval", 2.0, sizeof(double), 0, 0, closest_point_line_id);

	//vzm::CameraParameters cam_params;
	//vzm::GetCameraParameters(scene_id, cam_params, 0);
	//glm::fvec3 xyz_LT_view_up[3] = { (pos_closest_line[0] + pos_closest_line[1]) * 0.5f, __cv3__ cam_params.view, __cv3__ cam_params.up };
	float dist = glm::length(pos_closest_line[0] - pos_closest_line[1]);
	if (dist < 0.001f) dist = 0;
	//vzm::GenerateTextObject(__FP xyz_LT_view_up, to_string_with_precision(dist, 3) + "mm", font_size, true, false, dist_text_id);
	//vzm::ReplaceOrAddSceneObject(0, dist_text_id, obj_state_closest_point_line);
}

void MakeAngle(const int scene_id, const glm::fvec3& tool_tip2end_dir, const glm::fvec3& guide_dst2end_dir, const glm::fvec3& pos_dst_point, const float font_size, const float angle_tris_length, int& angle_tris_id, int& angle_text_id)
{
	//const float font_size = 30.f;
	const int num_angle_tris = 10;
	//const float angle_tris_length = 50.f;
	glm::fvec3 vec_ref = glm::normalize(glm::cross(guide_dst2end_dir, tool_tip2end_dir));
	float angle = glm::orientedAngle(guide_dst2end_dir, tool_tip2end_dir, vec_ref);
	//std::cout << angle << std::endl;
	std::vector<glm::fvec3> anlge_polygon_pos(num_angle_tris + 2);
	std::vector<glm::fvec3> anlge_polygon_clr(num_angle_tris + 2);
	anlge_polygon_pos[0] = pos_dst_point;
	anlge_polygon_clr[0] = glm::fvec3(1);
	std::vector<unsigned int> idx_prims(num_angle_tris * 3);
	for (int i = 0; i < num_angle_tris + 1; i++)
	{
		glm::fvec3 r_vec = glm::rotate(guide_dst2end_dir, angle / (float)num_angle_tris * (float)i, vec_ref);
		anlge_polygon_pos[1 + i] = pos_dst_point + r_vec * angle_tris_length;
		anlge_polygon_clr[1 + i] = glm::fvec3((float)i / (float)num_angle_tris, 0, 1.f - (float)i / (float)num_angle_tris);
		if (i < num_angle_tris)
		{
			idx_prims[3 * i + 0] = 0;
			idx_prims[3 * i + 1] = i + 1;
			idx_prims[3 * i + 2] = i + 2;
		}
	}
	//static int angle_tris_id = 0, angle_text_id = 0;
	vzm::GeneratePrimitiveObject(__FP anlge_polygon_pos[0], NULL, __FP anlge_polygon_clr[0], NULL, num_angle_tris + 2, (unsigned int*)&idx_prims[0], num_angle_tris, 3, angle_tris_id);
	//vzm::ObjStates obj_state_angle_tris;
	//obj_state_angle_tris.color[3] = 0.5f;
	//vzm::ReplaceOrAddSceneObject(0, angle_tris_id, obj_state_angle_tris);

	vzm::CameraParameters cam_params;
	vzm::GetCameraParameters(scene_id, cam_params, 1);
	glm::fvec3 xyz_LT_view_up[3] = { anlge_polygon_pos[1], __cv3__ cam_params.view, __cv3__ cam_params.up };
	if (angle * 180.f / glm::pi<float>() < 0.1) angle = 0;
	vzm::GenerateTextObject(__FP xyz_LT_view_up, to_string_with_precision(angle * 180.f / glm::pi<float>(), 3) + "вк", font_size, true, false, angle_text_id);
	vzm::ObjStates obj_state_angle_text;
	//vzm::ReplaceOrAddSceneObject(0, angle_text_id, obj_state_angle_text);
}

void MakeAngle2(const glm::fvec3& tool_tip2end_dir, const glm::fvec3& guide_dst2end_dir, const glm::fvec3& pos_dst_point, const float font_size, const float angle_tris_length, 
	int& angle_tris_id,
	const int scene0_id, int& angle_text0_id,
	const int scene1_id, int& angle_text1_id)
{
	//const float font_size = 30.f;
	const int num_angle_tris = 10;
	//const float angle_tris_length = 50.f;
	glm::fvec3 vec_ref = glm::normalize(glm::cross(guide_dst2end_dir, tool_tip2end_dir));

	float angle = glm::orientedAngle(guide_dst2end_dir, tool_tip2end_dir, vec_ref);
	if (angle > glm::pi<float>() * 0.5f)
	{
		angle = glm::pi<float>() - angle;
	}
	//std::cout << angle << std::endl;
	std::vector<glm::fvec3> anlge_polygon_pos(num_angle_tris + 2);
	std::vector<glm::fvec3> anlge_polygon_clr(num_angle_tris + 2);
	anlge_polygon_pos[0] = pos_dst_point;
	anlge_polygon_clr[0] = glm::fvec3(1);
	std::vector<unsigned int> idx_prims(num_angle_tris * 3);
	for (int i = 0; i < num_angle_tris + 1; i++)
	{
		glm::fvec3 r_vec = glm::rotate(guide_dst2end_dir, angle / (float)num_angle_tris * (float)i, vec_ref);
		anlge_polygon_pos[1 + i] = pos_dst_point + r_vec * angle_tris_length;
		anlge_polygon_clr[1 + i] = glm::fvec3((float)i / (float)num_angle_tris, 0, 1.f - (float)i / (float)num_angle_tris);
		if (i < num_angle_tris)
		{
			idx_prims[3 * i + 0] = 0;
			idx_prims[3 * i + 1] = i + 1;
			idx_prims[3 * i + 2] = i + 2;
		}
	}
	//static int angle_tris_id = 0, angle_text_id = 0;
	vzm::GeneratePrimitiveObject(__FP anlge_polygon_pos[0], NULL, __FP anlge_polygon_clr[0], NULL, num_angle_tris + 2, (unsigned int*)&idx_prims[0], num_angle_tris, 3, angle_tris_id);
	//vzm::ObjStates obj_state_angle_tris;
	//obj_state_angle_tris.color[3] = 0.5f;
	//vzm::ReplaceOrAddSceneObject(0, angle_tris_id, obj_state_angle_tris);

	for (int i = 0; i < 2; i++)
	{
		vzm::CameraParameters cam_params;
		vzm::GetCameraParameters(i == 0 ? scene0_id : scene1_id, cam_params, 1);
		glm::fvec3 xyz_LT_view_up[3] = { anlge_polygon_pos[1], __cv3__ cam_params.view, __cv3__ cam_params.up };
		if (angle * 180.f / glm::pi<float>() < 0.1) angle = 0;
		vzm::GenerateTextObject(__FP xyz_LT_view_up, to_string_with_precision(angle * 180.f / glm::pi<float>(), 3) + "вк", font_size, true, false, i == 0 ? angle_text0_id : angle_text1_id);
	}
	//vzm::ObjStates obj_state_angle_text;
	//vzm::ReplaceOrAddSceneObject(0, angle_text_id, obj_state_angle_text);
}

//void MakeTrackeffect(const int track_fade_num, const glm::fvec3& pos_dst, int& track_spheres_id, std::vector<glm::fvec3>& track_points)
void MakeTrackeffect(const int track_fade_num, const float min_move_dist, const glm::fvec3& pos_dst, int& track_spheres_id)
{
	//const int track_fade_num = 100;
	//static int track_spheres_id = 0;
	static std::vector<glm::fvec3> track_points;
	if (track_points.size() == 0 || glm::length(track_points[0] - pos_dst) > min_move_dist)
	{
		if (track_points.size() < track_fade_num)
		{
			track_points.push_back(pos_dst);
		}
		else
		{
			memcpy(&track_points[0], &track_points[1], sizeof(glm::fvec3) * (track_fade_num - 1));
			track_points[track_fade_num - 1] = pos_dst;
		}
		std::vector<glm::fvec4> trackspheres(track_points.size());
		for (int i = 0; i < (int)track_points.size(); i++)
		{
			trackspheres[i] = glm::fvec4(track_points[i], 0.001f + i * 0.005f / track_fade_num);
		}
		vzm::GenerateSpheresObject(__FP trackspheres[0], NULL, track_points.size(), track_spheres_id);
		//vzm::ObjStates obj_state_track_spheres;
		//vzm::ReplaceOrAddSceneObject(0, track_spheres_id, obj_state_track_spheres);
	}
}

void SetManualProbe(const std::string& dst_tool_name, const std::string& src_tool_name, const PROBE_MODE prob_mode, const int pos_idx, GlobalInfo& ginfo)
{
	if (pos_idx < 0 || pos_idx > 1 || prob_mode == PROBE_MODE::DEFAULT) return;

	bool is_tool_src_tracked = false, is_tool_dst_tracked = false;
	glm::fmat4x4 mat_srcfrm2ws, mat_dstfrm2ws;
	is_tool_src_tracked = ginfo.otrk_data.trk_info.GetLFrmInfo(src_tool_name, mat_srcfrm2ws);
	is_tool_dst_tracked = ginfo.otrk_data.trk_info.GetLFrmInfo(dst_tool_name, mat_dstfrm2ws);
	if (is_tool_src_tracked && is_tool_dst_tracked)
	{
		glm::fvec3 pos_pt_ws = tr_pt(mat_srcfrm2ws, glm::fvec3(0));
		glm::fmat4x4 mat_ws2dstfrm = glm::inverse(mat_dstfrm2ws);
		glm::fvec3 pos_pt_dstfrm = tr_pt(mat_ws2dstfrm, pos_pt_ws);

		std::string file_path = ginfo.custom_pos_file_paths["preset_path"];

		vector<Point3f>& custom_pos_list = ginfo.otrk_data.custom_pos_map[dst_tool_name];
		bool store_preset = false;
		if (prob_mode == PROBE_MODE::ONLY_PIN_POS)
		{
			custom_pos_list.clear();
			custom_pos_list.push_back(*(Point3f*)&pos_pt_dstfrm);

			file_path += "..\\Preset\\" + dst_tool_name + "_end.txt";
			store_preset = true;
		}
		else if (prob_mode == PROBE_MODE::ONLY_RBFRAME)
		{
			if (pos_idx == 0) custom_pos_list.clear();
			else if (pos_idx == 1 && custom_pos_list.size() == 2) custom_pos_list.pop_back();
			else if (pos_idx == 1 && custom_pos_list.size() == 0)
			{
				cout << "NO S POINT!!" << endl;
				return;
			}

			custom_pos_list.push_back(*(Point3f*)&pos_pt_dstfrm);

			if (pos_idx == 1)
			{
				file_path += "..\\Preset\\" + dst_tool_name + "_se.txt";
				store_preset = true;
			}
		}

		if (store_preset)
		{
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
	}
}

void SetCustomTools(const std::string& tool_name, const PROBE_MODE probe_mode, const GlobalInfo& ginfo, const glm::fvec3& tool_color, const bool visible)
{
	static map<std::string, int> tool_names;
	for (auto it : tool_names)
	{
		vzm::ObjStates tool_state;
		tool_state.is_visible = false;
		vzm::ReplaceOrAddSceneObject(ginfo.stg_scene_id, it.second, tool_state);
		vzm::ReplaceOrAddSceneObject(ginfo.rs_scene_id, it.second, tool_state);
		vzm::ReplaceOrAddSceneObject(ginfo.ws_scene_id, it.second, tool_state);
	}
	if (!visible) return;
	int& tool_id = tool_names[tool_name];
	int& tool_tip_id = tool_names[tool_name + "_tip"];

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

	glm::fmat4x4 mat_lfrm2ws;
	if (ginfo.otrk_data.trk_info.GetLFrmInfo(tool_name, mat_lfrm2ws))
	{
		glm::fvec3 pos_tool_tip = tr_pt(mat_lfrm2ws, glm::fvec3(0));
		glm::fvec3 tool_dir = glm::normalize(tr_vec(mat_lfrm2ws, glm::fvec3(0, 0, 1)));
		auto it = ginfo.otrk_data.custom_pos_map.find(tool_name);
		if (it != ginfo.otrk_data.custom_pos_map.end())
		{
			const vector<Point3f>& custom_pos_list = it->second;
			if (custom_pos_list.size() > 0)
			{
				if (probe_mode == ONLY_RBFRAME && custom_pos_list.size() == 2)
				{
					glm::fvec3 pos_s = *(glm::fvec3*)&custom_pos_list[0];
					pos_tool_tip = tr_pt(mat_lfrm2ws, pos_s);
					glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[1];
					tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
				}
				else if (probe_mode == ONLY_PIN_POS && custom_pos_list.size() == 1)
				{
					glm::fvec3 pos_e = *(glm::fvec3*)&custom_pos_list[0];
					tool_dir = glm::normalize(tr_pt(mat_lfrm2ws, pos_e) - pos_tool_tip);
				}
			}
		}
		register_tool_obj(pos_tool_tip, tool_dir, 0.2f, 0.002f, tool_color, tool_id, tool_tip_id);
	}
}

glm::fmat4x4 MatrixWS2CS(const glm::fvec3& pos_eye, const glm::fvec3& vec_view, const glm::fvec3& vec_up)
{
	using namespace glm;
	const dvec3& _pos_eye = pos_eye;
	const dvec3& _vec_up = vec_up;
	const dvec3& _vec_view = vec_view;

	dvec3 d3VecAxisZ = -_vec_view;
	d3VecAxisZ = glm::normalize(d3VecAxisZ);

	dvec3 d3VecAxisX = glm::cross(_vec_up, d3VecAxisZ);
	d3VecAxisX = glm::normalize(d3VecAxisX);

	dvec3 d3VecAxisY = glm::cross(d3VecAxisZ, d3VecAxisX);
	d3VecAxisY = glm::normalize(d3VecAxisY);

	fmat4x4 mat;

	mat[0][0] = d3VecAxisX.x;
	mat[0][1] = d3VecAxisY.x;
	mat[0][2] = d3VecAxisZ.x;
	mat[0][3] = 0;
	mat[1][0] = d3VecAxisX.y;
	mat[1][1] = d3VecAxisY.y;
	mat[1][2] = d3VecAxisZ.y;
	mat[1][3] = 0;
	mat[2][0] = d3VecAxisX.z;
	mat[2][1] = d3VecAxisY.z;
	mat[2][2] = d3VecAxisZ.z;
	mat[2][3] = 0;
	mat[3][0] = -glm::dot(d3VecAxisX, _pos_eye);
	mat[3][1] = -glm::dot(d3VecAxisY, _pos_eye);
	mat[3][2] = -glm::dot(d3VecAxisZ, _pos_eye);
	mat[3][3] = 1;

	return mat;
};

void Show_Window(const std::string& title, const int scene_id, const int cam_id, const std::string* ptext)
{
	static std::map<std::tuple<int, int>, std::tuple<uint, double>> timer_data;
	auto _data = timer_data[std::tuple<int, int>(scene_id, cam_id)];
	uint count = std::get<0>(_data);
	double times_sum = std::get<1>(_data);
	DWORD _t = GetTickCount();
	vzm::RenderScene(scene_id, cam_id);
	//times_sum += (GetTickCount() - _t) / 1000.;
	//std::cout << "rendering sec : " << times_sum / (double)(++count) << " s" << std::endl;
	times_sum = (GetTickCount() - _t) / 1000.;
	//std::cout << "rendering sec : " << times_sum << " s" << std::endl;
	unsigned char* ptr_rgba;
	float* ptr_zdepth;
	int w, h;
	if (vzm::GetRenderBufferPtrs(scene_id, &ptr_rgba, &ptr_zdepth, &w, &h, cam_id))
	{
		cv::Mat cvmat(h, w, CV_8UC4, ptr_rgba);
		//show the image
		if(ptext)
			cv::putText(cvmat, *ptext, cv::Point(3, 30), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 185, 255), 1, LineTypes::LINE_AA);

		cv::imshow(title, cvmat);
	}
}

void Show_Window(const std::string& title, const int scene_id, const int cam_id)
{
	Show_Window(title, scene_id, cam_id, NULL);
}

void Show_Window_with_Texts(const std::string& title, const int scene_id, const int cam_id, const std::string& text)
{
	Show_Window(title, scene_id, cam_id, &text);
}

void Show_Window_with_Info(const std::string& title, const int scene_id, const int cam_id, const GlobalInfo& ginfo)
{
	vzm::RenderScene(scene_id, cam_id);
	unsigned char* ptr_rgba;
	float* ptr_zdepth;
	int w, h;
	if (vzm::GetRenderBufferPtrs(scene_id, &ptr_rgba, &ptr_zdepth, &w, &h, cam_id))
	{
		cv::Mat cvmat(h, w, CV_8UC4, ptr_rgba);
		//show the image
		if (title == "Model VIEW")
		{
			cv::putText(cvmat, "Point : " + to_string((int)ginfo.model_ms_pick_pts.size()), cv::Point(3, 30), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 185, 255), 1, LineTypes::LINE_AA);
			
			glm::fmat4x4 mat_ws2ss;
			vzm::GetCamProjMatrix(scene_id, cam_id, __FP mat_ws2ss);
			for (int i = 0; i < (int)ginfo.model_ms_pick_pts.size(); i++)
			{
				glm::fvec3 pos_ss = tr_pt(mat_ws2ss, ginfo.model_ms_pick_pts[i]);
				cv::putText(cvmat, to_string(i), cv::Point(pos_ss.x, pos_ss.y), cv::FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 255, 0, 255), 1, LineTypes::LINE_AA);
			}
		}

		cv::imshow(title, cvmat);
	}
}

int GL_CLR_CHANNELS = 4;
void copy_back_ui_buffer(unsigned char* data_ui, unsigned char* data_render_bf, int w, int h, bool v_flib)
{
	// cpu mem ==> dataPtr
	int width_uibuf_pitch = w * 3;
	int width_fbbuf_pitch = w * GL_CLR_CHANNELS;
#pragma omp parallel for 
	for (int i = 0; i < h; i++)
		for (int j = 0; j < w; j++)
		{
			int y = v_flib ? (h - 1 - i) : i;

			unsigned int rgba;
			memcpy(&rgba, &data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0], sizeof(int));
			
			//unsigned char r = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0];
			//unsigned char g = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 1];
			//unsigned char b = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 2];
			//unsigned char a = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 3];

			unsigned char r = rgba & 0xFF;
			unsigned char g = (rgba >> 8) & 0xFF;
			unsigned char b = (rgba >> 16) & 0xFF;
			unsigned char a = (rgba >> 24) & 0xFF;

			if (a > 0)
			{
				unsigned int rgb;
				memcpy(&rgb, &data_ui[i * width_uibuf_pitch + j * 3 + 0], 3);

				//unsigned char _r = data_ui[i * width_uibuf_pitch + j * 3 + 0];
				//unsigned char _g = data_ui[i * width_uibuf_pitch + j * 3 + 1];
				//unsigned char _b = data_ui[i * width_uibuf_pitch + j * 3 + 2];

				unsigned char _r = rgb & 0xFF;
				unsigned char _g = (rgb >> 8) & 0xFF;
				unsigned char _b = (rgb >> 16) & 0xFF;

				float fa = (float)a / 255.f;
				float fr = (1.f - fa) * (float)_r + (float)r * fa;
				float fg = (1.f - fa) * (float)_g + (float)g * fa;
				float fb = (1.f - fa) * (float)_b + (float)b * fa;

				//data_ui[i * width_uibuf_pitch + j * 3 + 0] = (unsigned char)min((int)fr, (int)255);
				//data_ui[i * width_uibuf_pitch + j * 3 + 1] = (unsigned char)min((int)fg, (int)255);
				//data_ui[i * width_uibuf_pitch + j * 3 + 2] = (unsigned char)min((int)fb, (int)255);

				rgb = (unsigned char)min((int)fr, (int)255) | ((unsigned char)min((int)fg, (int)255) << 8) | ((unsigned char)min((int)fb, (int)255) << 16);
				memcpy(&data_ui[i * width_uibuf_pitch + j * 3 + 0], &rgb, 3);
			}
		}
};

void copy_back_ui_buffer_local(unsigned char* data_ui, int w, int h, unsigned char* data_render_bf, int w_bf, int h_bf, int offset_x, int offset_y, bool v_flib, bool smooth_mask, float _a, float _b, bool opaque_bg)
{
	auto alpha_mask = [&smooth_mask, &_a, &_b](float r) -> float
	{
		if (!smooth_mask) return 1.f;
		//const float _a = 0.2;
		return min(max((atan(_a * (r - _b)) + atan(_a * 100.f)) / (atan(_a * 1000.f) * 2.f), 0.f), 1.f) * 0.8f;
	};
	// cpu mem ==> dataPtr
	int width_uibuf_pitch = w * 3;
	int width_fbbuf_pitch = w_bf * GL_CLR_CHANNELS;
	glm::fvec2 _c = glm::fvec2(w_bf * 0.5, h_bf * 0.5);
#pragma omp parallel for 
	for (int i = 0; i < h_bf; i++)
		for (int j = 0; j < w_bf; j++)
		{
			min(offset_y + h_bf, h);
			min(offset_x + w_bf, w);//

			int y = v_flib ? (h - 1 - i) : i;

			unsigned int rgba;
			memcpy(&rgba, &data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0], sizeof(int));

			unsigned char r = rgba & 0xFF;
			unsigned char g = (rgba >> 8) & 0xFF;
			unsigned char b = (rgba >> 16) & 0xFF;
			unsigned char a = opaque_bg? 255 : (rgba >> 24) & 0xFF;

			if ((a > 0) && (j + offset_y < w) && (i + offset_y < h))
			{
				unsigned int rgb;
				int ui_x = j + offset_x;
				int ui_y = i + offset_y;
				memcpy(&rgb, &data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0], 3);

				//unsigned char _r = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0];
				//unsigned char _g = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 1];
				//unsigned char _b = data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 2];

				unsigned char _r = rgb & 0xFF;
				unsigned char _g = (rgb >> 8) & 0xFF;
				unsigned char _b = (rgb >> 16) & 0xFF;

				//float _a = alpha_mask(glm::length(_c - glm::fvec2(j, i)));
				int _x = min(min(i, j), min(w_bf - j, h_bf - i));
				float _a = alpha_mask((float)_x);
				//if (i % 20 == 1 && j % 20 == 1)
				//	cout << _a << ", ";
				float fa = (float)a / 255.f * _a;
				float fr = (1.f - fa) * (float)_r + (float)r * fa;
				float fg = (1.f - fa) * (float)_g + (float)g * fa;
				float fb = (1.f - fa) * (float)_b + (float)b * fa;

				//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0] = (unsigned char)min((int)fr, (int)255);
				//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 1] = (unsigned char)min((int)fg, (int)255);
				//data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 2] = (unsigned char)min((int)fb, (int)255);

				rgb = (unsigned char)min((int)fr, (int)255) | ((unsigned char)min((int)fg, (int)255) << 8) | ((unsigned char)min((int)fb, (int)255) << 16);
				memcpy(&data_ui[ui_y * width_uibuf_pitch + ui_x * 3 + 0], &rgb, 3);
			}
		}
	cout << endl;
};

#define PAIR_MAKE(P2D, P3D) std::pair<cv::Point2f, cv::Point3f>(cv::Point2f(P2D.x, P2D.y), cv::Point3f(P3D.x, P3D.y, P3D.z))
#define double_vec3(D) ((double*)D.data)[0], ((double*)D.data)[1], ((double*)D.data)[2]
bool CalibrteCamLocalFrame(const vector<glm::fvec2>& points_2d, const vector<glm::fvec3>& points_3dws, const glm::fmat4x4& mat_ws2clf,
	const float fx, const float fy, const float cx, const float cy, glm::fmat4x4& mat_rscs2clf, float* err, int* num_samples,
	vector<pair<Point2f, Point3f>>& pair_pts)//, const int img_w, const int img_h

{
	if (points_2d.size() == 0) return false;
	if (points_2d.size() != points_3dws.size()) return false;
	using namespace glm;

//	pair<Point2f, Point3f>& _pair = pair_pts

	vector<Point3f> points_buf_3d_clf;
	vector<Point2f> points_buf_2d;
	for (int i = 0; i < (int)pair_pts.size(); i++)
	{
		pair<Point2f, Point3f>& _pair = pair_pts[i];
		points_buf_2d.push_back(std::get<0>(_pair));
		points_buf_3d_clf.push_back(std::get<1>(_pair));
	}

	int num_incoming_pts = (int)points_3dws.size();
	for (int i = 0; i < num_incoming_pts; i++)
	{
		Point2f p2d = *(Point2f*)&points_2d[i];
		Point3f p3d = *(Point3f*)&tr_pt(mat_ws2clf, points_3dws[i]);

		points_buf_2d.push_back(p2d);
		points_buf_3d_clf.push_back(p3d);
	}

	if (num_samples) *num_samples = points_buf_2d.size();
	if (points_buf_2d.size() < 12)
	{
		for (int i = 0; i < (int)points_buf_2d.size(); i++)
		{
			Point2f p2d = *(Point2f*)&points_buf_2d[i];
			Point3f p3d = *(Point3f*)&points_buf_3d_clf[i];

			pair_pts.push_back(PAIR_MAKE(p2d, p3d));
		}
		return false;
	}

	Mat cam_mat = cv::Mat::zeros(3, 3, CV_64FC1); // intrinsic camera parameters
	cam_mat.at<double>(0, 0) = fx;       //      [ fx   0  cx ]
	cam_mat.at<double>(1, 1) = fy;       //      [  0  fy  cy ]
	cam_mat.at<double>(0, 2) = cx;       //      [  0   0   1 ]
	cam_mat.at<double>(1, 2) = cy;
	cam_mat.at<double>(2, 2) = 1;
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
	//cv::solvePnP(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_DLS);
	//cv::solvePnP(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, true, SOLVEPNP_ITERATIVE);
	cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_AP3P);
	cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, true, SOLVEPNP_ITERATIVE);

	float err_proj = 0;
	{
		vector<cv::Point2f> reprojectPoints;
		cv::projectPoints(Mat(*(vector<Point3f>*)&points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);
		float reproj_err_sum = 0.;
		reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*)&points_buf_2d)); //  default L2
		err_proj = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
		//cout << "PnP reprojection error : " << err_proj << " pixels, # of point pairs L " << points_buf_2d.size() << endl;
	}

	const float err_criterion = 3.f;
	Mat inliers_ids;
	if ((pair_pts.size() > 20 && err_proj > err_criterion)
		|| (pair_pts.size() > 50 && err_proj > 2.f)
		|| (pair_pts.size() > 100 && err_proj > 1.f))
	{
		float confidence = 0.9f;
		if (pair_pts.size() >= 100) confidence = 0.8f;
		cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, true, 5, err_criterion, confidence, inliers_ids, SOLVEPNP_ITERATIVE);
		cout << "# of inliers : " << inliers_ids.rows << endl;
		if (inliers_ids.rows > 0)
		{
			vector<Point3f> points_buf_3d_clf_tmp = points_buf_3d_clf;
			vector<Point2f> points_buf_2d_tmp = points_buf_2d;
			points_buf_3d_clf.clear();
			points_buf_2d.clear();
			for (int i = 0; i < inliers_ids.rows; i++)
			{
				int index = inliers_ids.at<int>(i, 0);
				//cout << i << ",";
				points_buf_3d_clf.push_back(points_buf_3d_clf_tmp[index]);
				points_buf_2d.push_back(points_buf_2d_tmp[index]);
			}
			//cout << endl;
		}
	}

	pair_pts.clear();
	for (int i = 0; i < (int)points_buf_2d.size(); i++)
	{
		Point2f p2d = *(Point2f*)&points_buf_2d[i];
		Point3f p3d = *(Point3f*)&points_buf_3d_clf[i];

		pair_pts.push_back(PAIR_MAKE(p2d, p3d));
	}

	//cv::calibrateCamera(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), Size(img_w, img_h), cam_mat, distCoeffs, rvec, tvec, 
	//	CALIB_USE_INTRINSIC_GUESS | CALIB_FIX_PRINCIPAL_POINT | CALIB_FIX_ASPECT_RATIO | CALIB_ZERO_TANGENT_DIST | CALIB_FIX_K1 | CALIB_FIX_K2 | CALIB_FIX_K3 | CALIB_FIX_K4 | CALIB_FIX_K5 | CALIB_FIX_K6, 
	//	TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));

	glm::fvec3 rot_v = glm::fvec3(double_vec3(rvec));
	float rad = glm::length(rot_v);

	fmat4x4 mat_clf2cvf = glm::translate(glm::fvec3(double_vec3(tvec)));
	mat_clf2cvf = glm::rotate(mat_clf2cvf, rad, rot_v / rad);
	fmat4x4 mat_cvf2clf = glm::inverse(mat_clf2cvf);

	fmat4x4 mat_cvf2rscs = glm::rotate((float)CV_PI, glm::fvec3(1, 0, 0));
	fmat4x4 mat_rscs2cvf = glm::inverse(mat_cvf2rscs);

	mat_rscs2clf = mat_cvf2clf * mat_rscs2cvf;

	if (err)
	{
		vector<cv::Point2f> reprojectPoints;
		cv::projectPoints(Mat(*(vector<Point3f>*)&points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints); 

		float reproj_err_sum = 0.;
		reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*)&points_buf_2d)); //  default L2
		*err = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
		cout << "PnP reprojection error : " << *err << " pixels, # of point pairs L " << points_buf_2d.size() << endl;
	}

	// TEST //
	//if (points_buf_2d.size() > 100)
	//{
	//	cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false);
	//
	//	vector<cv::Point2f> reprojectPoints;
	//	cv::projectPoints(Mat(*(vector<Point3f>*)&points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);
	//	float reproj_err_sum = 0.;
	//	reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*)&points_buf_2d)); //  default L2
	//	*err = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
	//	cout << "PnP Ransac reprojection error : " << *err << " pixels" << endl;
	//}

	return true;
};

void ComputeCameraStates(const glm::fmat4x4& mat_rscs2clf, // computed through calibration using CalibrteCamLocalFrame
	const glm::fmat4x4& mat_clf2ws, // at every frame, stereo IR cams gives this matrix by tracking rs's rigid IR markers
	vzm::CameraParameters& cam_state // only update CameraParameters::pos, up, view
	)
{
	using namespace glm;
	fvec3 pos_c_rscs(0);
	fvec3 vec_up_rscs(0, 1, 0);
	fvec3 vec_view_rscs(0, 0, -1);

	fmat4x4 mat_rscs2ws = mat_clf2ws * mat_rscs2clf;
	pos_c_rscs = tr_pt(mat_rscs2ws, pos_c_rscs);
	vec_up_rscs = normalize(tr_vec(mat_rscs2ws, vec_up_rscs));
	vec_view_rscs = normalize(tr_vec(mat_rscs2ws, vec_view_rscs));

	__cv3__ cam_state.pos = pos_c_rscs;
	__cv3__ cam_state.up = vec_up_rscs;
	__cv3__ cam_state.view = vec_view_rscs;
};

bool IsMeshModel(const int obj_id)
{
	return ((obj_id >> 24) & 0xFF) == 2;
}

bool GetSufacePickPos(glm::fvec3& pos_pick_ws, const int scene_id, const int cam_id, const bool use_pickmodel, const int x, const int y)
{
	if (use_pickmodel) // which means it is a primitive-type object
	{
		int pick_obj = 0;
		glm::fvec3 pos_pick;
		vzm::PickObject(pick_obj, __FP pos_pick, x, y, scene_id, cam_id);
		if (pick_obj != 0)
		{
			cout << "mesh model picked : " << pick_obj << endl;
			TESTOUT("world position : ", pos_pick);
			pos_pick_ws = pos_pick;
			return true;
		}
	}
	else 
	{
		glm::fvec3 pos_pick;
		if (vzm::Pick1stHitSurfaceUsingDepthMap(__FP pos_pick, x, y, 1000.f, scene_id, cam_id))
		{
			TESTOUT("world position : ", pos_pick);
			pos_pick_ws = pos_pick;
			return true;
		}
	}
	return false;
}

void Make_Buttons(const int screen_w, const int screen_h, std::map<RsTouchMode, ButtonState>& buttons)
{
	int w = screen_w;
	int h = screen_h;
	int bw = w / 5;
	int bh = 40;
	vector<RsTouchMode> btns_mode;
	btns_mode.push_back(RsTouchMode::None);
	btns_mode.push_back(RsTouchMode::Pick);
	btns_mode.push_back(RsTouchMode::Calib_TC);
	btns_mode.push_back(RsTouchMode::Calib_STG);
	btns_mode.push_back(RsTouchMode::Align);
	// create main buttons
	for (int i = 0; i < (int)btns_mode.size(); i++)
	{
		buttons[btns_mode[i]] = ButtonState(btns_mode[i], Rect(bw * i, 0, bw, bh), 0, true, EtoString(btns_mode[i]), false, Scalar(50, 50, 150, 200));
	}
#define ADD_SUBBTNS(MODE, RECT_INFO) buttons[MODE] = ButtonState(MODE, RECT_INFO, 0, false, EtoString(MODE), true, Scalar(150, 250, 150, 200))
	ADD_SUBBTNS(RsTouchMode::AR_Marker, Rect(bw * 1, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::DST_TOOL_E0, Rect(bw * 1, bh * 2, bw, bh));
	ADD_SUBBTNS(RsTouchMode::DST_TOOL_SE0, Rect(bw * 1, bh * 3, bw, bh));
	ADD_SUBBTNS(RsTouchMode::DST_TOOL_SE1, Rect(bw * 1, bh * 4, bw, bh));
	ADD_SUBBTNS(RsTouchMode::FIX_SCREW, Rect(bw * 1, bh * 5, bw, bh));
	ADD_SUBBTNS(RsTouchMode::ICP, Rect(bw * 4, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::Capture, Rect(bw * 4, bh * 2, bw, bh));
	ADD_SUBBTNS(RsTouchMode::Pair_Clear, Rect(bw * 2, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::Calib_STG2, Rect(bw * 3, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::STG_Pair_Clear, Rect(bw * 3, bh * 2, bw, bh));
}

void Draw_TouchButtons(cv::Mat img, const std::map<RsTouchMode, ButtonState>& buttons, const RsTouchMode touch_mode)
{
	for (auto it = buttons.begin(); it != buttons.end(); it++)
	{
		const ButtonState& btn = it->second;
		if (btn.is_activated)
		{
			Scalar bg = Scalar(150, 150, 150, 200);
			if (it->first == touch_mode) bg = btn.activated_bg;
			rectangle(img(btn.rect), Rect(0, 0, btn.rect.width, btn.rect.height), bg, -1);
			rectangle(img(btn.rect), Rect(0, 0, btn.rect.width, btn.rect.height), Scalar(0, 0, 0, 255), 2, LineTypes::LINE_AA);
			putText(img(btn.rect), btn.name, Point(btn.rect.width*0.1, btn.rect.height*0.4), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0, 255), 1, LineTypes::LINE_AA);
		}
	}
}