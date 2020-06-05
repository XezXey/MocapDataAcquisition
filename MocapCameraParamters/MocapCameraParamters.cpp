#include <windows.h>
#include <conio.h>
#include <direct.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <string.h>
#include <sys/types.h> 
#include <sys/stat.h> 

//Opencv libs
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <NPTrackingTools.h>

using namespace cv;
using namespace std;

// Local function prototypes
void CheckResult(NPRESULT result);
vector<Mat> getCameraExtrinsic();
Mat getCameraIntrinsic(int cameraIndex);
void writeToFile(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex);
void visualizeFrustum(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex);
int findCameraIndex(int targetCam);

// Global variables
fstream foutCalibPair;

class myListener : public cTTAPIListener
{
public:
    virtual void TTAPICameraConnected(int serialNumber)
    {
        printf("Camera Connected: Serial #%d\n", serialNumber);
    }
    virtual void TTAPICameraDisconnected(int serialNumber)
    {
        printf("Camera Disconnected: Serial #%d\n", serialNumber);
    }
};
int main(int argc, char* argv[]) {

    printf("== NaturalPoint Tracking Tools API Marker Sample =======---\n");
    printf("== (C) NaturalPoint, Inc.\n\n");

    printf("Initializing NaturalPoint Devices\n\n");

    if (TT_Initialize() != NPRESULT_SUCCESS)
    {
        printf("Unable to license Motive API\n");
        return 1;
    }
    else {
        printf("License is loaded.\n");
    }

    // Attach listener for camera notifications ==--
    myListener listener;
    TT_AttachListener(&listener);

    // Index of camera to calibrate
    int targetCam = 2;
    int targetCamIndex = findCameraIndex(targetCam);
    int frameCounter = 0;
    int saveFrameCounter = 0;
    //== Displaying all connected cameras ==//
    int totalCamera = TT_CameraCount();

    // Load a project file from the executable directory.
    printf("[#]Loading Profile: UserProfile.motive\n");
    CheckResult(TT_LoadProfile("C:\\Users\\VISTEC-DroneLab\\Desktop\\Mint\\Motive Profile.motive"));
    printf("[#]Before loading profile ===> CameraFrameRate : %d\n", TT_CameraFrameRate(targetCamIndex));

    // Load a calibration file from the executable directory
    printf("[#]Loading Calibration: Calibration.cal\n");
    CheckResult(TT_LoadCalibration("C:\\Users\\VISTEC-DroneLab\\Desktop\\Mint\\CalibrationResult 2020-06-03 10.cal"));
    
    printf("[#]After loading profile ===> CameraFrameRate : %d\n", TT_CameraFrameRate(targetCamIndex));
    printf("Number of camers : %d\n", TT_CameraCount());

    Sleep(2000);

    // Create folder to store image
    string outputFolder = "CaptureImage//";
    _mkdir(outputFolder.c_str());

	// Sample code for saving frame buffer from a camera (index 0)
	int reswidth;
	int resheight;
	int bytespan;

	// Camera Settings 
	int exposure = 2577;
	int intensity = 10;
	int grayScaleMode = 1;
	int threshold = 200;
	// Obtaining pixel resolution
	TT_SetCameraSettings(targetCamIndex, grayScaleMode, exposure, threshold, intensity);
	TT_CameraPixelResolution(targetCamIndex, reswidth, resheight);
	printf("Camera #%d:\tWidth:%d\tHeight:%d\n", targetCam, reswidth, resheight);

	// Allocating memory block for the buffer
	unsigned char* frameBuffer = (unsigned char*)std::malloc(reswidth * resheight * 1);
	// Defining stuff for opencv Mat datatype and imwrite
	bytespan = reswidth;
	Mat image;
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5);

    vector<Mat> cameraExtrinsic = getCameraExtrinsic();
    cout << "Camera " << targetCam << " : Extrinsic\n" << cameraExtrinsic[targetCamIndex] << endl;
    //Mat cameraIntrinsic = getCameraIntrinsic(targetCam);
    while (!_kbhit())
    {
        if (TT_Update() == NPRESULT_SUCCESS)
        {
            frameCounter++;

			printf("Frame : %d ===> Number of marker : %d\n", frameCounter, TT_FrameMarkerCount());
			for (int i = 0; i < TT_FrameMarkerCount(); i++)
			{
				float markerX = TT_FrameMarkerX(i);
				float markerY = TT_FrameMarkerY(i);
				float markerZ = TT_FrameMarkerZ(i);
				float camU;
				float camV;
				TT_CameraBackproject(targetCam, markerX, markerY, markerZ, camU, camV);
				printf("World : (%f, %f, %f) <===> ", markerX, markerY, markerZ);
				printf("Screen : (%f, %f)\n", camU, camV);
                //visualizeFrustum(markerX, markerY, markerZ, camU, camV, targetCam, frameCounter);
			}

			bool result = TT_CameraFrameBuffer(targetCam, reswidth, resheight, bytespan, 8, frameBuffer);
			if (result == true)
			{
				image = Mat(Size(reswidth, resheight), CV_8UC1, frameBuffer);
				imshow("FRAME", image);
				if (waitKey(1) == (char)115) {
                    saveFrameCounter++;
					cout << "[#] " << saveFrameCounter << " Images saved." << endl;
                    string filename = outputFolder + (string)"image" + to_string(saveFrameCounter) + string(".png");
					imwrite(filename, image, compression_params);
				}
            }
        }
    }
    destroyAllWindows();
	return 0;
}


void visualizeFrustum(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex) {
    //glBegin(GL_LINES);
    //glVertex2f(10, 10);
    //glVertex2f(20, 20);
    //glEnd();
}

double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
    const vector< vector< Point2f > >& imagePoints,
    const vector< Mat >& rvecs, const vector< Mat >& tvecs,
    const Mat& cameraMatrix, const Mat& distCoeffs) {
    vector< Point2f > imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    vector< float > perViewErrors;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i) {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
            distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float)std::sqrt(err * err / n);
        totalErr += err * err;
        totalPoints += n;
    }
    return std::sqrt(totalErr / totalPoints);
}

int findCameraIndex(int targetCam) {
    //Mapping between targetCam and real index
    int targetCamSerial = 0;
    switch (targetCam) {
		case 1:
            targetCamSerial = 86812;
            break;
		case 2:
            targetCamSerial = 86816;
            break;
		case 3:
            targetCamSerial = 86727;
            break;
		case 4:
            targetCamSerial = 86815;
            break;
		case 5:
            targetCamSerial = 86809;
            break;
		case 6:
            targetCamSerial = 86807;
            break;
		case 7:
            targetCamSerial = 86808;
            break;
		case 8:
            targetCamSerial = 86806;
            break;
    }
    //== Displaying all connected cameras ==//
    int totalCamera = TT_CameraCount();
    printf("Detected Cameras Serial Numbers:\n");
    for (int i = 0; i < totalCamera; i++)
    {
        if (targetCamSerial == TT_CameraSerial(i)) {
            return i;
        }
    }
    return 0;
}

Mat getCameraIntrinsic(int cameraIndex) {
    vector< vector< Point3f > > objectPoints;
    vector< vector< Point2f > > imagePoints;
	printf("Starting Calibration\n");
    Mat K;
    Mat D;
    int width;
    int height;
    vector< Mat > rvecs, tvecs;
    int frameCounter = 0;
    // Get the image resolution of the target camera
    TT_CameraPixelResolution(cameraIndex, width, height);

	// opens an existing csv file or creates a new file. 
	foutCalibPair.open("CalibrationPairwise.csv", ios::out | ios::app);
    // Looping to read the image points and object points for calibration until keyboard hitted.
    while (!_kbhit()) {
        if (TT_Update() == NPRESULT_SUCCESS)
        {
			vector< Point3f> objEachFrame;  // This vector will store the object point in each frame
			vector< Point2f> imageEachFrame;  // This vector will store the image point in each frame
            frameCounter++;
			printf("[#] Frame : %d ===> Number of marker : %d\n", frameCounter, TT_FrameMarkerCount());
			for (int i = 0; i < TT_FrameMarkerCount(); i++)
			{
				float markerX = TT_FrameMarkerX(i);
				float markerY = TT_FrameMarkerY(i);
				float markerZ = TT_FrameMarkerZ(i);
                markerZ = 0;
				float camU;
				float camV;
				TT_CameraBackproject(cameraIndex, markerX, markerY, markerZ, camU, camV);
                cout << "===> World : " << Point3f(markerX, markerY, markerZ) << endl;
                cout << "===> Screen : " << Point2f(camU, camV) << endl;
				objEachFrame.push_back(Point3f(markerX, markerY, markerZ));
				imageEachFrame.push_back(Point2f(camU, camV));
                
                writeToFile(markerX, markerY, markerZ, camU, camV, cameraIndex, frameCounter);
			}
            if (TT_FrameMarkerCount() > 0) {
                cout << objEachFrame << endl;
                cout << imageEachFrame << endl;
				objectPoints.push_back(objEachFrame);
				imagePoints.push_back(imageEachFrame);
            }
            if (frameCounter == 20) {
                break;
            }
        }
    }
    cout << Size(width, height);
    int flag = 0;
    //flag |= CV_CALIB_FIX_K4;
    //flag |= CV_CALIB_FIX_K5;

    calibrateCamera(objectPoints, imagePoints, Size(width, height), K, D, rvecs, tvecs);

    cout << "Calibration error: " << computeReprojectionErrors(objectPoints, imagePoints,  rvecs, tvecs, K, D) << endl;
    return K, D;
}

void writeToFile(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex) {
    foutCalibPair << markerX << ", "
        << markerY << ", "
        << markerZ << ", "
        << camU << ", "
        << camV << ", "
        << cameraIndex << ", "
        << frameIndex << ", "
        << "\n";
}

vector<Mat> getCameraExtrinsic() {
    /*
    This function return the camera extrinsic of all available in the system.
    */
    const int numberOfCamera = TT_CameraCount();
    vector<Mat> cameraExtrinsic;
    for (int i = 0; i < numberOfCamera; i++)
    {
        //===== Translation Matrix =====/
        float camX = TT_CameraXLocation(i);
        float camY = TT_CameraYLocation(i);
        float camZ = TT_CameraZLocation(i);
        vector<float> translationVector{ camX, camY, camZ };
		//===== Rotation Matrix =====//
        Mat eachExtrinsic(3, 4, CV_32F);
		for (int j = 0; j < 3; j++) {
            for (int k = 0; k < 3; k++) {
                eachExtrinsic.at<float>(j, k) = TT_CameraOrientationMatrix(i, k + (3 * j)), k + (3 * j);
                //printf("\t%f (index %d)", TT_CameraOrientationMatrix(i, k + (3 * j)), k + (3 * j));
            }
            eachExtrinsic.at<float>(j, 3) = translationVector[j];
            //printf("\n\n");
        }
        eachExtrinsic.convertTo(eachExtrinsic, CV_32F);
        cameraExtrinsic.push_back(eachExtrinsic);
    }
    // Display the Camera Extrinsic
	//printf("Camera Extrinsic: \n");
    for (int i = 0; i < numberOfCamera; i++) {
		//cout << cameraExtrinsic[i] << endl;
    }
    return cameraExtrinsic;
}

void CheckResult(NPRESULT result)   //== CheckResult function will display errors and ---
                                      //== exit application after a key is pressed =====---
{
    if (result != NPRESULT_SUCCESS)
    {
        // Treat all errors as failure conditions.
        printf("Error: %s\n\n(Press any key to continue)\n", TT_GetResultString(result));

        Sleep(20);
        exit(1);
    }
}