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

void process_image(int flipMode, int negMode, cv::Mat& t_img, cv::Mat& c_img)
{
	// Flip
	cv::Mat t;
	cv::Mat img;
	switch (flipMode)
	{
	case 1:
		cv::flip(t_img, t, 1);
		c_img = t.clone();
		break;
	case 2:
		cv::flip(c_img, img, 1);
		c_img = img.clone();
		break;
	case 3:
		cv::flip(t_img, t, 1);
		t_img = t;
		cv::flip(c_img, img, 1);
		c_img = img.clone();
		break;
	}
	// Negative
	switch (negMode)
	{
	case 1:
		cv::bitwise_not(t_img, t_img);
		break;
	case 2:
		cv::bitwise_not(c_img, c_img);
		break;
	case 3:
		cv::bitwise_not(t_img, t_img);
		cv::bitwise_not(c_img, c_img);
		break;
	}
}

bool get_stereo_images_list(std::string t_path, std::string c_path, stringvec& t_images, stringvec& c_images/*, stringvec& shared_images*/)
{
	// Clear variables
	t_images.clear();
	c_images.clear();

	stringvec t_files;
	stringvec c_files;
	stringvec nt_files;
	stringvec nc_files;
	read_directory(t_path, t_files);
	read_directory(c_path, c_files);
	std::string t_im = t_path;
	std::string c_im = c_path;
	t_im += "\\";
	c_im += "\\";

	if (t_files.size() == 0)
	{
		printf("No thermal images found.");
		return false;
	}
	else if (c_files.size() == 0)
	{
		printf("No color images found.");
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

	for (int i = 0; i < nt_files.size(); i++)
	{
		for (int w = 0; w < nc_files.size(); w++)
		{
			if ((t_files[i].compare(".") != 0 && c_files[w].compare(".") != 0)
				&& (t_files[i].compare("..") != 0 && c_files[w].compare("..") != 0))
			{
				if (nt_files[i].compare(nc_files[w]) == 0)
				{
					std::string tp;
					std::string cp;
					tp = t_im;
					cp = c_im;

					tp += t_files[i];
					cp += c_files[w];

					t_images.push_back(tp);
					c_images.push_back(cp);
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
	rms = cv::stereoCalibrate(objectPoints, imagePoints1, imagePoints2, cameraMatrix1, distCoeffs1, cameraMatrix2, distCoeffs2, imageSize, R, T, E, F, CV_CALIB_FIX_INTRINSIC);

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

// MAIN ---------------------------------------------------------------------------
int main(int argc, char* argv[])
{
	const std::string inputSettingsFile = argc > 1 ? argv[1] : "configuration.xml";

	// CONFIG
	std::string path_thermo = "";
	std::string path_color = "";
	std::string in_data_name;
	std::string out_data_name;
	int cd_resolution_mode = 0;
	int cd_imwidth = 1280;
	int cd_imheight = 720;
	int cd_fps = 30;
	int board_width;
	int board_height;
	float squareSize = 2.0;

	int flip_mode = 0;
	int negative_mode = 0;

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
	fs["InputPath"] >> in_data_name;
	fs["OutputPath"] >> out_data_name;
	fs["FlipMode"] >> flip_mode;
	fs["NegativeMode"] >> negative_mode;
	fs["BoardSize_Width"] >> board_width;
	fs["BoardSize_Height"] >> board_height;
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
	if (negative_mode < 0 && negative_mode > 1)
	{
		printf("Invalid flip mode.\n");
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

	// Check if images where found
	if (!get_stereo_images_list(path_thermo, path_color, thermo_images_path, color_images_path))
	{
		printf("No images found inside the folder.\n");
		system("pause");
		return -1;
	}

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

		//Image process
		// Modify images
		process_image(flip_mode, negative_mode, t_frame, c_frame);

		// convert to 8bit
		t_frame.convertTo(t_frame, CV_8UC1, 1 / 256.0);

		std::vector<cv::Point2f> t_pointBuf;
		std::vector<cv::Point2f> c_pointBuf;
		bool t_found;
		t_found = findCirclesGrid(t_frame, patternsize, t_pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);
		bool c_found;
		c_found = findCirclesGrid(c_frame, patternsize, c_pointBuf, cv::CALIB_CB_ASYMMETRIC_GRID);

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
		}
		if (!t_found && c_found)
		{
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", thermo_images_path[i].c_str());
		}
		if (!t_found && !c_found)
		{
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", thermo_images_path[i].c_str());
			printf("Can't detect %s image circles. Image will be descarted in extrinsics calibration.\n", color_images_path[i].c_str());
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
		
		char key = (char)cv::waitKey(25);

		if (key == 27)
		{
			break;
		}
	}

	cv::destroyAllWindows();
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
			cv::Mat t_frame;
			cv::Mat c_frame;
			
			t_frame = cv::imread(thermo_images_path[i], cv::IMREAD_ANYDEPTH);
			c_frame = cv::imread(color_images_path[i], cv::IMREAD_COLOR);
			
            process_image(flip_mode, negative_mode, t_frame, c_frame);
			// convert to 8bit
			t_frame.convertTo(t_frame, CV_8UC1, 1 / 256.0);

			cv::Mat registeredResult;
			bool dilatationC1 = false;

			cv::Mat tR, cR, tP, cP, Q;
			cv::Mat rgbdt;
			double alpha = 0.5;
			double beta = (1.0 - alpha);
			cv::Mat cc;
			/*cv::cvtColor(t_frame, cc, CV_GRAY2RGB);
			cc = c_frame.clone();*/
			cv::Rect tRoi, cRoi;

			cv::stereoRectify(in_thermal_camaeraMatrix, in_thermal_distCoeff, in_color_cameraMatrix, in_color_distCoeff, t_imageSize, R, T, tR, cR, tP, cP, Q, CV_CALIB_ZERO_DISPARITY, -1, c_imageSize, &tRoi, &cRoi);
			
			cv::Mat rmap[2][2];
			//Precompute maps for cv::remap()
			initUndistortRectifyMap(in_thermal_camaeraMatrix, in_thermal_distCoeff, tR, tP, t_imageSize, CV_16SC2, rmap[0][0], rmap[0][1]);
			initUndistortRectifyMap(in_color_cameraMatrix, in_color_distCoeff, cR, cP, c_imageSize, CV_16SC2, rmap[1][0], rmap[1][1]);

			cv::Mat canvas;
			double sf;
			int w, h;

			sf = 600. / MAX(c_imageSize.width, c_imageSize.height);
			w = cvRound(c_imageSize.width*sf);
			h = cvRound(c_imageSize.height*sf);
			canvas.create(h, w * 2, CV_8UC3);
			
			cv::Mat timg, cimg;
 			cv::remap(t_frame, timg, rmap[0][0], rmap[0][1], CV_INTER_LINEAR);
			cvtColor(t_frame, timg, cv::COLOR_GRAY2BGR);
			cv::remap(c_frame, cimg, rmap[1][0], rmap[1][1], CV_INTER_LINEAR);
			//cvtColor(c_frame, cimg, cv::COLOR_GRAY2BGR);

			t_frame = timg.clone();
			c_frame = cimg.clone();
				
			for (int j = 0; j < canvas.rows; j += 16)
				line(canvas, cv::Point(0, j), cv::Point(canvas.cols, j), cv::Scalar(0, 255, 0), 1, 8);
				
		
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
			

			/*cv::addWeighted(c_frame, alpha, cc, beta, 0.0, rgbdt);

			cv::namedWindow("Overlay", cv::WINDOW_AUTOSIZE);
			cv::imshow("Overlay", rgbdt);*/

			char key = (char)cv::waitKey(0);

			if (key == 27)
			{
				break;
			}
			
		}
	}
	cv::destroyAllWindows();
	system("pause");
	return -1;
}
