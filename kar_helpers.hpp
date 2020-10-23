#pragma once

#include <iostream>
#include <windows.h>

#include <queue>
#include <bitset>
#include <string>

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
			cv::putText(cvmat, *ptext, cv::Point(3, 30), cv::FONT_HERSHEY_DUPLEX, 0.5, CV_RGB(255, 185, 255));

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

void copy_back_ui_buffer_local(unsigned char* data_ui, int w, int h, unsigned char* data_render_bf, int w_bf, int h_bf, int offset_x, int offset_y, bool v_flib)
{
	// cpu mem ==> dataPtr
	int width_uibuf_pitch = w * 3;
	int width_fbbuf_pitch = w_bf * GL_CLR_CHANNELS;
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
			unsigned char a = (rgba >> 24) & 0xFF;

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

				float fa = (float)a / 255.f;
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
		pair_pts.push_back(PAIR_MAKE(p2d, p3d));
	}

	if (num_samples) *num_samples = points_buf_2d.size();
	if (pair_pts.size() < 12) return false;

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
	cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, false, SOLVEPNP_DLS);
	cv::solvePnP(points_buf_3d_clf, points_buf_2d, cam_mat, distCoeffs, rvec, tvec, true, SOLVEPNP_ITERATIVE);
	//cv::solvePnPRansac(Mat(*(vector<Point3f>*)&points_buf_3d_clf), Mat(*(vector<Point2f>*)&points_buf_2d), cam_mat, distCoeffs, rvec, tvec, false, 5, 1.f, 0.98, noArray(), SOLVEPNP_DLS);

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
		bytes_size = (sizeof(glm::fmat4x4) + sizeof(bool) + sizeof(char) * 100) * num_lfrms + (sizeof(float) * 2 + (128/8)) * num_mks + 4 * 2; // last 4 * 2 means num_mks, num_lfrms
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
	int cb_spheres_id;
	int mks_spheres_id;
	int rs_lf_axis_id, probe_lf_axis_id; // lf means local frame
	vector<Point3f> calib_3d_pts;
	vector<pair<Point2f, Point3f>> tc_calib_pt_pairs;
	vector<pair<Point2f, Point3f>> stg_calib_pt_pairs;
	bitset<128> stg_calib_mk_cid;
	vector<int> calib_trial_rs_cam_frame_ids;
	vector<int> mk_pickable_sphere_ids;

	OpttrkData()
	{
		stg_calib_mk_cid = 0;
		cb_spheres_id = 0;
		mks_spheres_id = 0;
		rs_lf_axis_id = probe_lf_axis_id = 0;
		obj_state.emission = 0.4f;
		obj_state.diffusion = 0.6f;
		obj_state.specular = 0.2f;
		obj_state.sp_pow = 30.f;
		__cv4__ obj_state.color = glm::fvec4(1.f, 1.f, 1.f, 1.f);
		__cm4__ obj_state.os2ws = glm::fmat4x4();
	}
};

ENUM(RsTouchMode, None, Pick, Calib_TC, PIN_ORIENTATION, Calib_STG, Align, ICP, Capture, Pair_Clear, STG_Pair_Clear)

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
	map<int, glm::fvec3> vzmobjid2pos;

	RsTouchMode touch_mode;
	bool skip_call_render;
	OpttrkData otrk_data;
	bool is_calib_rs_cam;
	bool is_calib_stg_cam;

	glm::fvec3 pos_probe_pin;

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
	vector<glm::fvec3> model_ws_pick_pts;
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
	string guide_path;		// 20200818 숭실대 guide 경로때문에 변수하나 추가했어요

	// rs cam ui buttons
	map<RsTouchMode, ButtonState> rs_buttons;

	GlobalInfo()
	{
		touch_mode = RsTouchMode::None;
		skip_call_render = false;
		is_calib_rs_cam = false;
		is_calib_stg_cam = false;
		model_ms_obj_id = 0;
		model_ws_obj_id = 0;
		captured_model_ms_point_id = 0;
		is_modelaligned = false;
		rs_pc_id = 0;
		model_volume_id = 0;

		// SSU
		brain_ms_obj_id = 0;
		brain_ws_obj_id = 0;
		ventricle_ms_obj_id = 0;
		ventricle_ws_obj_id = 0;
	}
};

#define TESTOUT(NAME, P) {cout << NAME << P.x << ", " << P.y << ", " << P.z << endl;}

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
		buttons[btns_mode[i]] = ButtonState(btns_mode[i], Rect(bw * i, 0, bw, bh), 0, true, EtoString(btns_mode[i]), false, Scalar(50, 50, 150));
	}
#define ADD_SUBBTNS(MODE, RECT_INFO) buttons[MODE] = ButtonState(MODE, RECT_INFO, 0, false, EtoString(MODE), true, Scalar(150, 250, 150))
	ADD_SUBBTNS(RsTouchMode::ICP, Rect(bw * 4, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::Capture, Rect(bw * 4, bh * 2, bw, bh));
	ADD_SUBBTNS(RsTouchMode::Pair_Clear, Rect(bw * 2, bh, bw, bh));
	ADD_SUBBTNS(RsTouchMode::STG_Pair_Clear, Rect(bw * 3, bh, bw, bh));
}

void Draw_TouchButtons(cv::Mat img, const std::map<RsTouchMode, ButtonState>& buttons, const RsTouchMode touch_mode)
{
	for (auto it = buttons.begin(); it != buttons.end(); it++)
	{
		const ButtonState& btn = it->second;
		if (btn.is_activated)
		{
			Scalar bg = Scalar(150, 150, 150);
			if (it->first == touch_mode) bg = btn.activated_bg;
			rectangle(img(btn.rect), Rect(0, 0, btn.rect.width, btn.rect.height), bg, -1);
			rectangle(img(btn.rect), Rect(0, 0, btn.rect.width, btn.rect.height), Scalar(0, 0, 0), 1);
			putText(img(btn.rect), btn.name, Point(btn.rect.width*0.1, btn.rect.height*0.4), FONT_HERSHEY_PLAIN, 1, Scalar(0, 0, 0));
		}
	}
}