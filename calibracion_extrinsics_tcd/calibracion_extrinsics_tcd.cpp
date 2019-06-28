#include "stdio.h"
#include <Windows.h>
#include <filesystem>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iterator>

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/rgbd.hpp"

#include "librealsense2/rs.hpp"

#pragma warning(disable : 4996)

typedef std::vector<std::string> stringvec;

// FUNCTIONS
void read_directory(const std::string& name, stringvec& v)
{
	std::string pattern(name);
	pattern.append("\\*");
	WIN32_FIND_DATA data;
	HANDLE hFind;
	if ((hFind = FindFirstFile(pattern.c_str(), &data)) != INVALID_HANDLE_VALUE) {
		do {
			v.push_back(data.cFileName);
		} while (FindNextFile(hFind, &data) != 0);
		FindClose(hFind);
	}
}

std::string exePath()
{
	char buffer[MAX_PATH];
	GetModuleFileName(NULL, buffer, MAX_PATH);
	std::string::size_type pos = std::string(buffer).find_last_of("\\/");
	return std::string(buffer).substr(0, pos);
}

bool dirExists(const std::string& dirName_in)
{
	DWORD ftyp = GetFileAttributesA(dirName_in.c_str());
	if (ftyp == INVALID_FILE_ATTRIBUTES)
		return false;  //something is wrong with your path!

	if (ftyp & FILE_ATTRIBUTE_DIRECTORY)
		return true;   // this is a directory!

	return false;    // this is not a directory!
}

bool get_images_path(std::string path, stringvec& images)
{
	stringvec files;
	read_directory(path, files);
	std::string im = path;
	im += "\\";
	for (int i = 0; i < files.size(); i++)
	{
		std::size_t found_png = files[i].find(".png");
		std::size_t found_jpg = files[i].find(".jpg");
		if (found_png != std::string::npos || found_jpg != std::string::npos)
		{
			std::string p = im;
			p += files[i];
			images.push_back(p);
		}

	}
	if (files.size() == 0)
	{
		return false;
	}
	return true;
}

void process_image(int flipMode, int patternTemp, int patternColor, cv::Mat& t_img, cv::Mat& c_img, cv::Mat& d_img)
{
	// Flip
	cv::Mat t;
	cv::Mat img;
	cv::Mat dep;
	switch (flipMode)
	{
	case 1:
		cv::flip(t_img, t, 1);
		c_img = t.clone();
		break;
	case 2:
		cv::flip(c_img, img, 1);
		cv::flip(d_img, dep, 1);
		c_img = img.clone();
		d_img = dep.clone();
		break;
	case 3:
		cv::flip(t_img, t, 1);
		t_img = t;
		cv::flip(c_img, img, 1);
		c_img = img.clone();
		cv::flip(d_img, dep, 1);
		d_img = dep.clone();
		break;
	}
	// Negative images
	if (patternTemp == 1)
	{
		cv::bitwise_not(t_img, t_img);
	}
	if (patternColor == 0)
	{
		cv::bitwise_not(c_img, c_img);
	}
}

bool get_stereo_images_list(std::string t_path, std::string c_path, std::string d_path, stringvec& t_images, stringvec& c_images, stringvec& d_images/*, stringvec& shared_images*/)
{
	// Clear variables
	t_images.clear();
	c_images.clear();
	d_images.clear();

	stringvec t_files;
	stringvec c_files;
	stringvec d_files;
	stringvec nt_files;
	stringvec nc_files;
	stringvec nd_files;
	read_directory(t_path, t_files);
	read_directory(c_path, c_files);
	read_directory(d_path, d_files);

	std::string t_im = t_path;
	std::string c_im = c_path;
	std::string d_im = d_path;
	t_im += "\\";
	c_im += "\\";
	d_im += "\\";

	if (t_files.size() == 0)
	{
		printf("No thermal images found.\n");
		return false;
	}
	if (c_files.size() == 0)
	{
		printf("No color images found.\n");
		return false;
	}
	if (d_files.size() == 0)
	{
		printf("No depth images found.\n");
		return false;
	}

	for (int i = 0; i < t_files.size(); i++)
	{
		std::size_t found_png = t_files[i].find(".png");
		std::size_t found_jpg = t_files[i].find(".jpg");
		if (found_png != std::string::npos || found_jpg != std::string::npos)
		{
			std::string p;
			p = t_files[i].substr(0, t_files[i].size() - 4);
			p.erase(0, 2);
			nt_files.push_back(p);
		}

	}

	for (int i = 0; i < c_files.size(); i++)
	{
		std::size_t found_png = c_files[i].find(".png");
		std::size_t found_jpg = c_files[i].find(".jpg");
		if (found_png != std::string::npos || found_jpg != std::string::npos)
		{
			std::string p;
			p = c_files[i].substr(0, c_files[i].size() - 4);
			p.erase(0, 2);
			nc_files.push_back(p);
		}
	}

	for (int i = 0; i < d_files.size(); i++)
	{
		std::size_t found_png = c_files[i].find(".png");
		std::size_t found_jpg = c_files[i].find(".jpg");
		if (found_png != std::string::npos || found_jpg != std::string::npos)
		{
			std::string p;
			p = d_files[i].substr(0, d_files[i].size() - 4);
			p.erase(0, 2);
			nd_files.push_back(p);
		}
	}

	for (int i = 0; i < nt_files.size(); i++)
	{
		for (int w = 0; w < nc_files.size(); w++)
		{
			for (int j = 0; j < nd_files.size(); j++)
			{
				if (nt_files[i].compare(nc_files[w]) == 0 && nt_files[i].compare(nd_files[j]) == 0)
				{
					std::string tp;
					std::string cp;
					std::string dp;
					tp = t_im;
					cp = c_im;
					dp = d_im;

					tp += "t_";
					tp += nt_files[i];
					tp += ".png";

					cp += "c_";
					cp += nc_files[w];
					cp += ".png";

					dp += "d_";
					dp += nd_files[j];
					dp += ".png";

					t_images.push_back(tp);
					c_images.push_back(cp);
					d_images.push_back(dp);
					/*shared_images.push_back(nt_files[i]);*/
				}
			
			}
		}
	}
	return true;
}

bool readCameraParams(std::string inputSettingsFile, int& thermal_image_width, int& thermal_image_height, int& color_image_width, int& color_image_height,
	cv::Mat& thermal_camera_matrix, cv::Mat& thermal_dist_coeff, cv::Mat& color_camera_matrix, cv::Mat& color_dist_coeff)
{
	// READ CONFIG VALUES
	cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		printf("ERROR: Could not open the configuration file: %s\n", inputSettingsFile.c_str());
		return false;
	}

	fs["thermal_image_Width"] >> thermal_image_width;
	fs["thermal_image_Height"] >> thermal_image_height;
	fs["color_image_Width"] >> color_image_width;
	fs["color_image_Height"] >> color_image_height;

	fs["Thermal_Camera_Matrix"] >> thermal_camera_matrix;
	fs["Thermal_Distortion_Coefficients"] >> thermal_dist_coeff;

	fs["Color_Camera_Matrix"] >> color_camera_matrix;
	fs["Color_Distortion_Coefficients"] >> color_dist_coeff;

	fs.release();
	return true;
}

static double computeReprojectionErrors(const std::vector<std::vector<cv::Point3f> >& objectPoints,
	const std::vector<std::vector<cv::Point2f> >& imagePoints,
	const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
	const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs,
	std::vector<float>& perViewErrors)
{
	std::vector<cv::Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i)
	{
		projectPoints(cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcBoardCornerPositions(cv::Size boardSize, float squareSize, std::vector<cv::Point3f>& corners/*,
	Settings::Pattern patternType*/ /*= Settings::CHESSBOARD*/)
{
	corners.clear();

	for (int i = 0; i < boardSize.height; i++)
		for (int j = 0; j < boardSize.width; j++)
			corners.push_back(cv::Point3f(float((2 * j + i % 2)*squareSize), float(i*squareSize), 0));
}

static bool runCalibration(cv::Size boardSize, float squareSize, cv::Size& imageSize, cv::Mat& cameraMatrix, cv::Mat& distCoeffs,
	std::vector<std::vector<cv::Point2f> > imagePoints, std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
	std::vector<float>& reprojErrs, double& totalAvgErr)
{

	cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

	distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

	std::vector<std::vector<cv::Point3f>> objectPoints(1);
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]/*, cv::CALIB_CB_ASYMMETRIC_GRID*/);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	double rms = cv::calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix, distCoeffs, rvecs, tvecs);

	std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}

static bool runStereoCalibration(cv::Size boardSize, float squareSize, cv::Size& imageSize, cv::Mat& cameraMatrix1, cv::Mat& distCoeffs1, cv::Mat& cameraMatrix2, cv::Mat& distCoeffs2,
	std::vector<std::vector<cv::Point2f> > imagePoints1, std::vector<std::vector<cv::Point2f> > imagePoints2, cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F, double& rms)
{
	
	std::vector<std::vector<cv::Point3f>> objectPoints(1);
	calcBoardCornerPositions(boardSize, squareSize, objectPoints[0]/*, cv::CALIB_CB_ASYMMETRIC_GRID*/);

	objectPoints.resize(imagePoints1.size(), objectPoints[0]);

	//Find intrinsic and extrinsic camera parameters
	rms = cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, /*CV_CALIB_USE_INTRINSIC_GUESS*/ CV_CALIB_FIX_INTRINSIC);

	std::cout << "Re-projection error reported by calibrateCamera: " << rms << std::endl;

	bool ok1 = checkRange(cameraMatrix1) && checkRange(distCoeffs1);
	bool ok2 = checkRange(cameraMatrix2) && checkRange(distCoeffs2);

	bool ok;
	if (ok1 && ok2)
	{
		ok = true;
	}

	return ok;
}

// Print camera parameters to the output file
static void saveCameraParams(std::string outputFileName, cv::Size& t_imageSize, cv::Mat& t_cameraMatrix, cv::Mat& t_distCoeffs, const std::vector<std::vector<cv::Point2f>>& t_imagePoints,
	cv::Size& c_imageSize, cv::Mat& c_cameraMatrix, cv::Mat& c_distCoeffs, const std::vector<std::vector<cv::Point2f>>& c_imagePoints, cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F, double& rms)
{
	cv::FileStorage fs(outputFileName, cv::FileStorage::WRITE);

	time_t tm;
	time(&tm);
	struct tm *t2 = localtime(&tm);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_Time" << buf;

	if (t_imagePoints.size() == c_imagePoints.size())
		fs << "nrOfFrames" << (int)c_imagePoints.size();
	fs << "thermal_image_Width" << t_imageSize.width;
	fs << "thermal_image_Height" << t_imageSize.height;
	fs << "color_image_Width" << c_imageSize.width;
	fs << "color_image_Height" << c_imageSize.height;

	fs << "Thermal_Camera_Matrix" << t_cameraMatrix;
	fs << "Thermal_Distortion_Coefficients" << t_distCoeffs;
	fs << "Color_Camera_Matrix" << c_cameraMatrix;
	fs << "Color_Distortion_Coefficients" << c_distCoeffs;

	fs << "R" << R;
	fs << "T" << T;
	fs << "E" << E;
	fs << "F" << F;

	fs << "Total_Reprojection_Error" << rms;

	fs.release();
}

bool runCalibrationAndSave(std::string outputFileName, cv::Size boardSize, float squareSize, cv::Size t_imageSize, cv::Mat&  t_cameraMatrix, cv::Mat& t_distCoeffs, std::vector<std::vector<cv::Point2f>> t_imagePoints, 
	cv::Size& c_imageSize, cv::Mat& c_cameraMatrix, cv::Mat& c_distCoeffs, std::vector<std::vector<cv::Point2f>> c_imagePoints,
	cv::Mat& R, cv::Mat& T, cv::Mat& E, cv::Mat& F, double& rms)
{
	std::vector<cv::Mat> rvecs, tvecs;
	std::vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runStereoCalibration(boardSize, squareSize, t_imageSize, t_cameraMatrix, t_distCoeffs, c_cameraMatrix, c_distCoeffs,
		t_imagePoints, c_imagePoints, R, T, E, F, rms);

	std::cout << (ok ? "Calibration succeeded" : "Calibration failed")
		<< ". rms_error = " << rms << std::endl;

	if (ok)
		saveCameraParams(outputFileName, t_imageSize, t_cameraMatrix, t_distCoeffs, t_imagePoints,
			c_imageSize, c_cameraMatrix, c_distCoeffs, c_imagePoints, R, T, E, F, rms);
		return ok;
}
void pRegistration(const cv::Mat& inputDataC1,
	const cv::Matx33f& cameraMatrixC1,
	const cv::Matx33f& cameraMatrixC2,
	const cv::Mat_<float>& distCoeffC2,
	const cv::Mat& R, const cv::Mat& T,
	const cv::Size imagePlaneC2,
	const bool depthDilatationC1,
	cv::Mat& registeredResultC1);

static cv::Scalar randomColor(cv::RNG& rng)
{
	int icolor = (unsigned)rng;
	return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}


//void computeC2MC1(const cv::Mat &R1, const cv::Mat &tvec1, const cv::Mat &R2, const cv::Mat &tvec2,
//	cv::Mat &R_1to2, cv::Mat &tvec_1to2);

// MAIN ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	const std::string inputSettingsFile = argc > 1 ? argv[1] : "configuration.xml";

	// CONFIG
	std::string path_thermo = "";
	std::string path_color = "";
	std::string path_depth = "";
	std::string in_data_name;
	std::string out_data_name;
	int cd_resolution_mode = 0;
	int cd_imwidth = 1280;
	int cd_imheight = 720;
	int cd_fps = 30;
	int board_width;
	int board_height;
	float squareSize = 20.0;

	int flip_mode = 0;
	int pattern_temp = 0;
	int pattern_color = 0;

	bool show_overlay = false;

	int count_img = 0;
	int count_out = 0;

	// READ CONFIG VALUES
	cv::FileStorage fs(inputSettingsFile, cv::FileStorage::READ); // Read the settings
	if (!fs.isOpened())
	{
		printf("ERROR: Could not open the configuration file: %s\n", inputSettingsFile.c_str());
		system("pause");
		return -1;
	}

	fs["PathThermoImages"] >> path_thermo;
	fs["PathColorImages"] >> path_color;
	fs["PathDepthImages"] >> path_depth;
	fs["InputPath"] >> in_data_name;
	fs["OutputPath"] >> out_data_name;
	fs["FlipMode"] >> flip_mode;
	fs["PatternTemp"] >> pattern_temp;
	fs["PatternColor"] >> pattern_color;
	fs["BoardSize_Width"] >> board_width;
	fs["BoardSize_Height"] >> board_height;
	fs["ShowOverlay"] >> show_overlay;
	fs.release();

	// if no path defined, follow output_images folder structure
	if (path_thermo == "")
	{
		path_thermo = exePath();
		path_thermo += "\\output_images\\thermo";
	}
	if (path_color == "")
	{
		path_color = exePath();
		path_color += "\\output_images\\color";
	}
	if (path_depth == "")
	{
		path_depth = exePath();
		path_depth += "\\output_images\\depth";
	}
	if (in_data_name.empty())
	{
		printf("No input file path/name defined.\n");
		system("pause");
		return -1;
	}
	if (out_data_name.empty())
	{
		printf("No output file path/name defined.\n");
		system("pause");
		return -1;
	}
	switch (cd_resolution_mode) // Check resolution
	{
	case 0:
		cd_imwidth = 1280;
		cd_imheight = 720;
		break;
	case 1:
		cd_imwidth = 640;
		cd_imheight = 480;
		break;
	case 2:
		cd_imwidth = 848;
		cd_imheight = 480;
		break;
	default:
		printf("Resolution mode not recogniced. Set the resolution mode in the configuration file.\n");
		return -1;
		break;
	}
	if (cd_fps < 0) // Check fps
	{
		printf("Invalid FPS.\n");
		system("pause");
		return -1;
	}
	if (flip_mode < 0 && flip_mode > 1) // Check flip mode
	{
		printf("Invalid flip mode.\n");
		system("pause");
		return -1;
	}
	if (pattern_temp < 0 && pattern_temp > 1)
	{
		printf("Invalid pattern mode.\n");
		system("pause");
		return -1;
	}
	if (pattern_color < 0 && pattern_color > 1)
	{
		printf("Invalid pattern color.\n");
		system("pause");
		return -1;
	}
	if (board_width <= 0 || board_height <= 0)
	{
		printf("Invalid negative mode.\n");
		system("pause");
		return -1;
	}
	cv::Size patternsize(board_height, board_width);

	// Input file data variables
	int in_thermal_image_width;
	int in_thermal_image_height;
	int in_color_image_width;
	int in_color_image_height;
	cv::Mat in_thermal_camaeraMatrix;
	cv::Mat in_thermal_distCoeff;
	cv::Mat in_color_cameraMatrix;
	cv::Mat in_color_distCoeff;

	// read intrinsics file
	if (!readCameraParams(in_data_name,in_thermal_image_width, in_thermal_image_height, in_color_image_width, in_color_image_height,
		in_thermal_camaeraMatrix, in_thermal_distCoeff, in_color_cameraMatrix, in_color_distCoeff))
	{
		printf("Can not read the input data of the intrinsics parameter files.");
		system("pause");
		return -1;
	}
	
	// Check if paths exists
	if (!dirExists(path_thermo))
	{
		printf("Path of thermal images do not exist.");
		system("pause");
		return -1;
	}

	// Variables
	stringvec thermo_images_path;
	stringvec color_images_path;
	stringvec depth_images_path;
	

	// Check if images where found
	if (!get_stereo_images_list(path_thermo, path_color, path_depth, thermo_images_path, color_images_path, depth_images_path))
	{
		printf("No images found inside the folder.\n");
		system("pause");
		return -1;
	}
	const int n = thermo_images_path.size();
	std::vector<bool> circles_detected;

	std::vector<std::vector<cv::Point2f>> t_imagePoints;
	std::vector<std::vector<cv::Point2f>> c_imagePoints;
	cv::Size t_imageSize(in_thermal_image_width, in_thermal_image_height), c_imageSize(in_color_image_width, in_color_image_height);
	cv::Mat R, T, E, F;
	double rms_error;
	bool calib_done = false;

	// MAIN LOOP
	printf
	(
		"Starting the aplication ...\n"
		"- Press ESC to stop.\n\n"
	);

	for (int i = 0;; i++)
	{
		cv::Mat t_frame;
		cv::Mat c_frame;
		cv::Mat d_frame;
		if (i < thermo_images_path.size())
		{
			t_frame = cv::imread(thermo_images_path[i], cv::IMREAD_ANYDEPTH);
			if (t_frame.size() != t_imageSize)
			{
				printf("The thermal image grabbed from the folder and the defined thermal image size in the intrinsics file are not equal.\n");
				system("pause");
				return -1;
			}
		}
		if (i < color_images_path.size())
		{
			c_frame = cv::imread(color_images_path[i], cv::IMREAD_COLOR);
			if (c_frame.size() != c_imageSize)
			{
				printf("The color image grabbed from the folder and the defined thermal image size in the intrinsics file are not equal.\n");
				system("pause");
				return -1;
			}
		}
		if (i < depth_images_path.size())
		{
			d_frame = cv::imread(depth_images_path[i], cv::IMREAD_ANYDEPTH);
			if (d_frame.size() != c_imageSize)
			{
				printf("The depth image grabbed from the folder and the defined thermal image size in the intrinsics file are not equal.\n");
				system("pause");
				return -1;
			}
		}
		
		if (t_frame.empty() || thermo_images_path.size() == i)          // If no more images then run calibration, save and stop loop.
		{
			if (t_imagePoints.size() > 0)
				printf("Starting camera calibration ...\n");
			calib_done = runCalibrationAndSave(out_data_name, patternsize, squareSize, t_imageSize, in_thermal_camaeraMatrix, in_thermal_distCoeff, t_imagePoints,
				c_imageSize, in_color_cameraMatrix, in_color_distCoeff, c_imagePoints, R, T, E, F, rms_error);
			if (!calib_done)
			{
				printf("Error while doing extrinsics calibration.\n");
				system("pause");
				return -1;
			}
			break;
		}

		t_imageSize = t_frame.size();

		cv::Mat t_img = t_frame.clone();
		cv::Mat c_img = c_frame.clone();
		cv::Mat d_img = d_frame.clone();
		// convert thermal 16U to 8U
		t_img.convertTo(t_img, CV_8UC1, 1 / 256.0);
		// increase contrast color img
		c_img.convertTo(c_img, -1, 1.75, 0);

		//Image process
		process_image(flip_mode, 0, 1, t_frame, c_frame, d_frame);
		process_image(flip_mode, pattern_temp, pattern_color, t_img, c_img, d_img);

		std::vector<cv::Point2f> t_pointBuf;
		std::vector<cv::Point2f> c_pointBuf;
		bool t_found;
		t_found = findCirclesGrid(t_img, patternsize, t_pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
		bool c_found;
		c_found = findCirclesGrid(c_img, patternsize, c_pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);

		if (!t_found || !c_found)
		{
			circles_detected.push_back(false);
		}
		else
		{
			circles_detected.push_back(true);
		}
		if (t_found && c_found) // If cercles found in both images
		{
			t_imagePoints.push_back(t_pointBuf);
			cv::drawChessboardCorners(t_frame, patternsize, cv::Mat(t_pointBuf), t_found);
			c_imagePoints.push_back(c_pointBuf);
			cv::drawChessboardCorners(c_frame, patternsize, cv::Mat(c_pointBuf), c_found);
		}
		if (t_found && !c_found) 
		{
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", color_images_path[i].c_str());
			count_out++;
		}
		if (!t_found && c_found)
		{
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", thermo_images_path[i].c_str());
			count_out++;
		}
		if (!t_found && !c_found)
		{
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", thermo_images_path[i].c_str());
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", color_images_path[i].c_str());
			count_out++;
		}

		if (t_found && c_found)
		{
			count_img++;
		}

		if (t_frame.empty() == false)
		{
			cv::namedWindow("Thermal camera", cv::WINDOW_AUTOSIZE);
			char txt[50];
			sprintf(txt, "%d | %d : detected | not detected", count_img, count_out);
			cv::putText(t_frame, txt, cv::Point(5, 480 - 5), cv::FONT_HERSHEY_DUPLEX, 0.85, 0xffff, 1);
			cv::imshow("Thermal camera", t_frame);
		}
		if (c_frame.empty() == false)
		{
			cv::namedWindow("Color camera", cv::WINDOW_AUTOSIZE);
			char txt[50];
			//sprintf(txt, "%d Set detected", count_img);
			sprintf(txt, "%d | %d : detected | not detected", count_img, count_out);
			cv::putText(c_frame, txt, cv::Point(5, cd_imheight - 5), cv::FONT_HERSHEY_DUPLEX, 0.85, cv::Scalar(0, 0, 255), 1);
			cv::imshow("Color camera", c_frame);
		}
		
		char key = (char)cv::waitKey(25);

		if (key == 27)
		{
			break;
		}
	}

	cv::destroyAllWindows();
	
	if (show_overlay)
	{
		int subst = 0;
		if (calib_done)
		{
			printf("\nStart Registered image visualization ...\n");
			printf
			(
				"- Press ESC to exit.\n"
				"- Press any key to visualize the next image.\n"
			);
			for (int i = 0; i < thermo_images_path.size(); i++)
			{
				cv::Mat t_frame, c_frame, d_frame;

				t_frame = cv::imread(thermo_images_path[i], cv::IMREAD_ANYDEPTH);
				c_frame = cv::imread(color_images_path[i], cv::IMREAD_COLOR);
				d_frame = cv::imread(depth_images_path[i], cv::IMREAD_ANYDEPTH);

				cv::Mat t_img = t_frame.clone();
				cv::Mat c_img = c_frame.clone();
				cv::Mat d_img = d_frame.clone();
				// convert thermal 16U to 8U
				t_img.convertTo(t_img, CV_8UC1, 1 / 256.0);
				// increase contrast color img
				c_img.convertTo(c_img, -1, 1.75, 0);

				//Image process
				process_image(flip_mode, 0, 1, t_frame, c_frame, d_frame);
				process_image(flip_mode, pattern_temp, pattern_color, t_img, c_img, d_img);

				cv::Mat registeredResult;
				bool dilatationC1 = true;
				cv::Mat Rt;
				cv::Mat o = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
				cv::hconcat(R, T, Rt);
				cv::vconcat(Rt, o, Rt);
				//cv::rgbd::registerDepth(in_color_cameraMatrix, in_thermal_camaeraMatrix, in_thermal_distCoeff, Rt, d_frame, t_frame.size(), registeredResult, dilatationC1);
				//pRegistration(d_frame, in_color_cameraMatrix, in_thermal_camaeraMatrix, in_thermal_distCoeff, R, T, t_frame.size(), dilatationC1, registeredResult);

				if (circles_detected[i])
				{
					int tot = i + subst;
					cv::Mat H = cv::findHomography(t_imagePoints[tot], c_imagePoints[tot]);
					std::cout << "H:\n" << H << std::endl;

					cv::Mat img1_warp;
					cv::Mat akan;
					warpPerspective(t_frame, img1_warp, H, c_frame.size());
					img1_warp.convertTo(img1_warp, CV_8U, 1 / 256.0);
					cv::cvtColor(img1_warp, img1_warp, CV_GRAY2RGB);

					double alpha = 0.5;
					double beta = (1.0 - alpha);
					cv::addWeighted(img1_warp, alpha, c_frame, beta, 0.0, akan);
					akan.convertTo(akan, -1, 1, 10);
					akan.convertTo(akan, -1, 2, 0);

					cv::namedWindow("w", cv::WINDOW_AUTOSIZE);
					cv::imshow("w", akan);
				}
				else
				{
					subst--;
				}

				if (t_frame.empty() == false)
				{
					cv::namedWindow("Thermal camera", cv::WINDOW_AUTOSIZE);
					cv::imshow("Thermal camera", t_frame);
				}
				if (c_frame.empty() == false)
				{
					cv::namedWindow("Color camera", cv::WINDOW_AUTOSIZE);
					cv::imshow("Color camera", c_frame);
				}

				char key = (char)cv::waitKey(0);

				if (key == 27)
				{
					break;
				}

			}
		}
	}
	
	cv::destroyAllWindows();
	system("pause");
	return -1;
}









///////////////////////////////////////////////////////////////////////////////////

	// Our three input types have a different value for a depth pixel with no depth
template<typename DepthDepth>
inline DepthDepth
noDepthSentinelValue()
{
	return 0;
}

template<>
inline float
noDepthSentinelValue<float>()
{
	return std::numeric_limits<float>::quiet_NaN();
}

template<>
inline double
noDepthSentinelValue<double>()
{
	return std::numeric_limits<double>::quiet_NaN();
}

///////////////////////////////////////////////////////////////////////////////////

	// Testing for depth pixels with no depth isn't straightforward for NaN values. We
	// need to specialize the equality check for floats and doubles.
template<typename DepthDepth>
inline bool
isEqualToNoDepthSentinelValue(const DepthDepth &value)
{
	return value == noDepthSentinelValue<DepthDepth>();
}

template<>
inline bool
isEqualToNoDepthSentinelValue<float>(const float &value)
{
	return cvIsNaN(value) != 0;
}

template<>
inline bool
isEqualToNoDepthSentinelValue<double>(const double &value)
{
	return cvIsNaN(value) != 0;
}

///////////////////////////////////////////////////////////////////////////////////


   // When using the unsigned short representation, we'd like to round the values to the nearest
   // integer value. The float/double representations don't need to be rounded
template<typename DepthDepth>
inline DepthDepth
floatToInputDepth(const float &value)
{
	return (DepthDepth)value;
}

template<>
inline unsigned short
floatToInputDepth<unsigned short>(const float &value)
{
	return (unsigned short)(value + 0.5);
}



//registerDepth.cpp
template<typename DepthDepth> void pRegistration(const cv::Mat_<DepthDepth>& inputDataC1,
	const cv::Matx33f& cameraMatrixC1,
	const cv::Matx33f& cameraMatrixC2,
	const cv::Mat_<float>& distCoeffC2,
	const cv::Mat& R, const cv::Mat& T,
	const cv::Size imagePlaneC2,
	const bool depthDilatationC1,
	cv::Mat& registeredResultC1)
{
	// Rigid-body transform
	cv::Mat rbtRgb2Depth;
	cv::Mat o = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
	cv::hconcat(R, T, rbtRgb2Depth);
	cv::vconcat(rbtRgb2Depth, o, rbtRgb2Depth);

	const float inputDepthToMetersScale = 1.0;
	// Create output Mat of the correct type, filled with an initial value indicating no depth
	registeredResultC1 = cv::Mat_<DepthDepth>(imagePlaneC2, noDepthSentinelValue<DepthDepth>());

	// Figure out whether we'll have to apply a distortion
	bool hasDistortion = (countNonZero(distCoeffC2) > 0);

	// A point (i,j,1) will have to be converted to 3d first, by multiplying it by K.inv()
	// It will then be transformed by rbtRgb2Depth.
	// Finally, it will be projected into the external camera via cameraMatrixC2 and
	// its distortion coefficients. If there is no distortion in the external camera, we
	// can linearly chain all three operations together.

	cv::Matx44f K = cv::Matx44f::zeros();
	for (unsigned char j = 0; j < 3; ++j)
		for (unsigned char i = 0; i < 3; ++i)
			K(j, i) = cameraMatrixC1(j, i);
	K(3, 3) = 1;

	cv::Matx44f initialProjection;
	if (hasDistortion)
	{
		// The projection into the external camera will be done separately with distortion
		initialProjection = rbtRgb2Depth * K.inv();
	}
	else
	{
		// No distortion, so all operations can be chained
		initialProjection = cv::Matx44f::zeros();
		for (unsigned char j = 0; j < 3; ++j)
			for (unsigned char i = 0; i < 3; ++i)
				initialProjection(j, i) = cameraMatrixC2(j, i);
		initialProjection(3, 3) = 1;

		initialProjection = initialProjection * rbtRgb2Depth * K.inv();
	}

	// Apply the initial projection to the input depth
	cv::Mat_<cv::Point3f> transformedCloud;
	{
		cv::Mat_<cv::Point3f> point_tmp(imagePlaneC2);

		for (int j = 0; j < point_tmp.rows; ++j)
		{
			const DepthDepth *inputDataC1Ptr = inputDataC1[j];

			cv::Point3f *point = point_tmp[j];
			for (int i = 0; i < point_tmp.cols; ++i, ++inputDataC1Ptr, ++point)
			{
				float rescaled_depth = float(*inputDataC1Ptr) * inputDepthToMetersScale;

				// If the DepthDepth is of type unsigned short, zero is a sentinel value to indicate
				// no depth. CV_32F and CV_64F should already have NaN for no depth values.
				if (rescaled_depth == 0)
				{
					rescaled_depth = std::numeric_limits<float>::quiet_NaN();
				}

				point->x = i * rescaled_depth;
				point->y = j * rescaled_depth;
				point->z = rescaled_depth;
			}
		}

		perspectiveTransform(point_tmp, transformedCloud, initialProjection);
	}

	std::vector<cv::Point2f> transformedAndProjectedPoints(transformedCloud.cols);
	const float metersToInputUnitsScale = 1 / inputDepthToMetersScale;
	const cv::Rect registeredResultC1Bounds(cv::Point(), imagePlaneC2);

	for (int y = 0; y < transformedCloud.rows; y++)
	{
		if (hasDistortion)
		{

			// Project an entire row of points with distortion.
			// Doing this for the entire image at once would require more memory.
			projectPoints(transformedCloud.row(y),
				cv::Vec3f(0, 0, 0),
				cv::Vec3f(0, 0, 0),
				cameraMatrixC2,
				distCoeffC2,
				transformedAndProjectedPoints);

		}
		else
		{

			// With no distortion, we just have to dehomogenize the point since all major transforms
			// already happened with initialProjection.
			cv::Point2f *point2d = &transformedAndProjectedPoints[0];
			const cv::Point2f *point2d_end = point2d + transformedAndProjectedPoints.size();
			const cv::Point3f *point3d = transformedCloud[y];
			for (; point2d < point2d_end; ++point2d, ++point3d)
			{
				point2d->x = point3d->x / point3d->z;
				point2d->y = point3d->y / point3d->z;
			}

		}

		const cv::Point2f *outputProjectedPoint = &transformedAndProjectedPoints[0];
		const cv::Point3f *p = transformedCloud[y], *p_end = p + transformedCloud.cols;


		for (; p < p_end; ++outputProjectedPoint, ++p)
		{



			// Skip this one if there isn't a valid depth
			const cv::Point2f projectedPixelFloatLocation = *outputProjectedPoint;
			if (cvIsNaN(projectedPixelFloatLocation.x))
				continue;

			//Get integer pixel location
			const cv::Point2i projectedPixelLocation = projectedPixelFloatLocation;

			// Ensure that the projected point is actually contained in our output image
			if (!registeredResultC1Bounds.contains(projectedPixelLocation))
				continue;

			// Go back to our original scale, since that's what our output will be
			// The templated function is to ensure that integer values are rounded to the nearest integer
			const DepthDepth cloudDepth = floatToInputDepth<DepthDepth>(p->z*metersToInputUnitsScale);

			DepthDepth& outputDepth = registeredResultC1.at<DepthDepth>(projectedPixelLocation.y, projectedPixelLocation.x);


			// Occlusion check
			if (isEqualToNoDepthSentinelValue<DepthDepth>(outputDepth) || (outputDepth > cloudDepth))
				outputDepth = cloudDepth;


			// If desired, dilate this point to avoid holes in the final image
			if (depthDilatationC1)
			{

				// Choosing to dilate in a 2x2 region, where the original projected location is in the bottom right of this
				// region. This is what's done on PrimeSense devices, but a more accurate scheme could be used.
				const cv::Point2i dilatedProjectedLocations[3] = { cv::Point2i(projectedPixelLocation.x - 1, projectedPixelLocation.y),
															  cv::Point2i(projectedPixelLocation.x    , projectedPixelLocation.y - 1),
															  cv::Point2i(projectedPixelLocation.x - 1, projectedPixelLocation.y - 1) };

				for (int i = 0; i < 3; i++) {

					const cv::Point2i& dilatedCoordinates = dilatedProjectedLocations[i];

					if (!registeredResultC1Bounds.contains(dilatedCoordinates))
						continue;

					DepthDepth& outputDilatedDepth = registeredResultC1.at<DepthDepth>(dilatedCoordinates.y, dilatedCoordinates.x);

					// Occlusion check
					if (isEqualToNoDepthSentinelValue(outputDilatedDepth) || (outputDilatedDepth > cloudDepth))
						outputDilatedDepth = cloudDepth;

				}

			} // depthDilatationC1

		} // iterate cols
	} // iterate rows
}