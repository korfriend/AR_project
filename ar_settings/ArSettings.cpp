#include "ArSettings.h"
#include <string>
#include <map>

using namespace std;
using namespace arsettings;

rs2::context ctx;
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
rs2::pipeline pipe;
rs2::config cfg;
rs2_intrinsics rgb_intrinsics;
rs2_intrinsics depth_intrinsics;

bool is_initialized = false;

void InitializeRealsense(const bool use_depthsensor, int rs_w, int rs_h)
{
	//for (auto&& dev : ctx.query_devices())
	//	serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));
	//cout << serials[0] << endl; // 839112061828 // 415
	//cout << serials[1] << endl; // 819312071259 // 430
	serials["RS_RBS"] = string("839112061828");
	serials["EYE"] = string("819312071259");

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

	pipe = rs2::pipeline(ctx);
	cfg.enable_device(serials["RS_RBS"]);
	cfg.enable_stream(RS2_STREAM_DEPTH, 848, 480, RS2_FORMAT_Z16, 60); // Enable default depth
	if(!use_depthsensor)
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
	}
	is_initialized = true;
}

void RunRsThread()
{

}