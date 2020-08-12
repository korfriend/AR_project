#pragma once

#include <iostream>
#include <windows.h>

#include <queue>

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

#define __cv3__ *(glm::fvec3*)
#define __cv4__ *(glm::fvec4*)
#define __cm4__ *(glm::fmat4x4*)

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

void Register_CamModel(const int scene_id, const glm::fmat4x4& mat_frm2ws, const string cam_label, const int cam_id)
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
			cv::putText(cvmat, *ptext, cv::Point(3, 30), cv::FONT_HERSHEY_DUPLEX, 1.0, CV_RGB(255, 185, 255));

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

int GL_CLR_CHANNELS = 4;
void copy_back_ui_buffer(unsigned char* data_ui, unsigned char* data_render_bf, int w, int h, bool v_flib)
{
	// cpu mem ==> dataPtr
	int width_uibuf_pitch = w * 3;
	int width_fbbuf_pitch = w * GL_CLR_CHANNELS;
	for (int i = 0; i < h; i++)
		for (int j = 0; j < w; j++)
		{
			int y = v_flib ? (h - 1 - i) : i;
			unsigned char r = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 0];
			unsigned char g = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 1];
			unsigned char b = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 2];
			unsigned char a = data_render_bf[y * width_fbbuf_pitch + j * GL_CLR_CHANNELS + 3];
			if (a > 0)
			{
				unsigned char _r = data_ui[i * width_uibuf_pitch + j * 3 + 0];
				unsigned char _g = data_ui[i * width_uibuf_pitch + j * 3 + 1];
				unsigned char _b = data_ui[i * width_uibuf_pitch + j * 3 + 2];

				float fa = (float)a / 255.f;
				float fr = (1.f - fa) * (float)_r + (float)r * fa;
				float fg = (1.f - fa) * (float)_g + (float)g * fa;
				float fb = (1.f - fa) * (float)_b + (float)b * fa;

				data_ui[i * width_uibuf_pitch + j * 3 + 0] = (byte)min((int)fr, (int)255);
				data_ui[i * width_uibuf_pitch + j * 3 + 1] = (byte)min((int)fg, (int)255);
				data_ui[i * width_uibuf_pitch + j * 3 + 2] = (byte)min((int)fb, (int)255);
			}
		}
};


enum CALIB_STATE
{
	INITIALIZE,
	UPDATE
};

#define double_vec3(D) ((double*)D.data)[0], ((double*)D.data)[1], ((double*)D.data)[2]
bool CalibrteCamLocalFrame(const vector<glm::fvec2>& points_2d, const vector<glm::fvec3>& points_3dws, const glm::fmat4x4& mat_ws2clf,
	const float fx, const float fy, const float cx, const float cy, CALIB_STATE cmode, glm::fmat4x4& mat_rscs2clf, float* err, int* num_samples)
{
	if (points_2d.size() == 0) return false;
	if (points_2d.size() != points_3dws.size()) return false;
	using namespace glm;

	static vector<fvec3> points_buf_3d_clf;
	static vector<fvec2> points_buf_2d;
	if (cmode == INITIALIZE)
	{
		points_buf_3d_clf.clear();
		points_buf_2d.clear();
	}

	int num_incoming_pts = (int)points_3dws.size();
	vector<fvec3> points_3dclf(num_incoming_pts);
	for (int i = 0; i < num_incoming_pts; i++)
	{
		points_3dclf[i] = tr_pt(mat_ws2clf, points_3dws[i]);
	}

	points_buf_2d.insert(points_buf_2d.end(), points_2d.begin(), points_2d.end());
	points_buf_3d_clf.insert(points_buf_3d_clf.end(), points_3dclf.begin(), points_3dclf.end());

	if (num_samples) *num_samples = points_buf_2d.size();
	if (points_buf_2d.size() < 12) return false;

	Mat cam_mat = cv::Mat::zeros(3, 3, CV_64FC1); // intrinsic camera parameters
	cam_mat.at<double>(0, 0) = fx;       //      [ fx   0  cx ]
	cam_mat.at<double>(1, 1) = fy;       //      [  0  fy  cy ]
	cam_mat.at<double>(0, 2) = cx;       //      [  0   0   1 ]
	cam_mat.at<double>(1, 2) = cy;
	cam_mat.at<double>(2, 2) = 1;
	cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);    // vector of distortion coefficients
	cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
	cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output translation vector
	cv::solvePnP(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_DLS);
	//cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false, 5, 1.f, 0.98, noArray(), SOLVEPNP_DLS);

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
		cout << "PnP reprojection error : " << *err << " pixels" << endl;
	}

	// TEST //
	if (points_buf_2d.size() > 100)
	{
		cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false);

		vector<cv::Point2f> reprojectPoints;
		cv::projectPoints(Mat(*(vector<Point3f>*)&points_buf_3d_clf), rvec, tvec, cam_mat, cv::noArray(), reprojectPoints);
		float reproj_err_sum = 0.;
		reproj_err_sum = cv::norm(Mat(reprojectPoints), Mat(*(vector<Point2f>*)&points_buf_2d)); //  default L2
		*err = sqrt(reproj_err_sum * reproj_err_sum / points_buf_2d.size());
		cout << "PnP Ransac reprojection error : " << *err << " pixels" << endl;
	}

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
	//"rs_cam"
	glm::fmat4x4 mat_rbcam2ws;
	bool is_detected_rscam;

	//"probe"
	glm::fmat4x4 mat_probe2ws;
	bool is_detected_probe;

	//"ss_tool_v1"
	glm::fmat4x4 mat_tfrm2ws;
	bool is_detected_sstool;

	//"ss_head"
	glm::fmat4x4 mat_headfrm2ws;
	bool is_detected_sshead;

	//"breastbody"
	glm::fmat4x4 mat_bodyfrm2ws;
	bool is_detected_brbody;

	std::vector<float> mk_xyz_list;
	std::vector<float> mk_residue_list;

	bool is_updated;
	track_info() { is_updated = false; }

	glm::fvec3 GetProbePinPoint()
	{
		return tr_pt(mat_probe2ws, glm::fvec3(0));
	}

	glm::fvec3 GetMkPos(int idx)
	{
		if (idx < 0 || idx >= mk_xyz_list.size() / 3) return glm::fvec3(0);
		return ((glm::fvec3*)&mk_xyz_list[0])[idx];
	}
};

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

ENUM(RsMouseMode, NONE, ADD_CALIB_POINTS, GATHERING_POINTS, PIN_ORIENTATION)

struct GlobalInfo
{
	map<int, glm::fvec3> vzmobjid2mkid;

	RsMouseMode rs_ms_mode;
	bool skip_main_thread;
	OpttrkData otrk_data;
	bool is_calib_cam;

	glm::fvec3 pos_probe_pin;

	// model related
	bool is_meshmodel;
	int model_obj_id;
	vector<glm::fvec3> model_pick_pts;
	bool align_matching_model;
	glm::fmat4x4 mat_match_model2ws;
	int gathered_model_point_id;

	int rs_pc_id;

	int brain_obj_id, ventricle_obj_id;

	// scene definition
	int ws_scene_id; // arbitrary integer
	int rs_scene_id; // arbitrary integer
	int model_scene_id; // arbitrary integer
	int csection_scene_id; // arbitrary integer

	// cv window name
	string window_name_rs_view;
	string window_name_ws_view;
	string window_name_ms_view;
	string window_name_hm_view;
	
	// file path
	string optrack_calib;
	string optrack_env;
	string cb_positions;
	string sst_positions;
	string model_path;

	GlobalInfo()
	{
		rs_ms_mode = NONE;
		skip_main_thread = false;
		is_calib_cam = false;
		model_obj_id = 0;
		gathered_model_point_id = 0;
		is_meshmodel = true;
		align_matching_model = false;
		rs_pc_id = 0;

		brain_obj_id = 0;
		ventricle_obj_id = 0;
	}
};

#define TESTOUT(NAME, P) {cout << NAME << P.x << ", " << P.y << ", " << P.z << endl;}