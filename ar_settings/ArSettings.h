#pragma once

#ifndef __dojostatic
#define __dojostatic extern "C" __declspec(dllexport)
#endif

#include <iostream>
#include <fstream>
#include <sstream>
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

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <librealsense2/rsutil.h>

namespace arsettings
{
}