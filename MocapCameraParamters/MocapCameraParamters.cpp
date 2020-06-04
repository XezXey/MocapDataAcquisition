#include <windows.h>
#include <conio.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <gl/GL.h>
#include <gl/GLU.h>

//Opencv libs
#include <opencv2/core/core.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <NPTrackingTools.h>

using namespace cv;
using namespace std;

// Local function prototypes
void CheckResult(NPRESULT result);
vector<Mat> getCameraExtrinsic();
Mat getCameraIntrinsic(int cameraIndex);
void writeToFile(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex);
void visualizeFrustum(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex);

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

    // Do an update to pick up any recently-arrived cameras.
    TT_Update();

    printf("[#]Before loading profile ===> CameraFrameRate : %d\n", TT_CameraFrameRate(1));
    // Load a project file from the executable directory.
    printf("[#]Loading Profile: UserProfile.motive\n");
    CheckResult(TT_LoadProfile("C:\\Users\\VISTEC-DroneLab\\Desktop\\Mint\\Motive Profile.motive"));


    // Load a calibration file from the executable directory
    printf("[#]Loading Calibration: Calibration.cal\n");
    CheckResult(TT_LoadCalibration("C:\\Users\\VISTEC-DroneLab\\Desktop\\Mint\\CalibrationResult 2020-06-03 10.cal"));
    
    printf("[#]After loading profile ===> CameraFrameRate : %d\n", TT_CameraFrameRate(1));
    printf("Number of camers : %d\n", TT_CameraCount());

    Sleep(2000);

    printf("[#]Finding Pair of 2D-3D points for camera calibration\n");
    // Index of camera to calibrate
    int targetCam = 2;

    int frameCounter = 0;
    // Poll API data until the user hits a keyboard key.

    vector<Mat> cameraExtrinsic = getCameraExtrinsic();
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
				float cam2dx;
				float cam2dy;
				TT_CameraBackproject(targetCam, markerX, markerY, markerZ, cam2dx, cam2dy);
				printf("World : (%f, %f, %f) <===> ", markerX, markerY, markerZ);
				printf("Screen : (%f, %f)\n", cam2dx, cam2dy);
			}
        }
    }

	return 0;
}

void visualizeFrustum(float markerX, float markerY, float markerZ, float camU, float camV, int cameraIndex, int frameIndex) {
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
                printf("\t%f (index %d)", TT_CameraOrientationMatrix(i, k + (3 * j)), k + (3 * j));
            }
            eachExtrinsic.at<float>(j, 3) = translationVector[j];
            printf("\n\n");
        }
        eachExtrinsic.convertTo(eachExtrinsic, CV_32F);
        cameraExtrinsic.push_back(eachExtrinsic);
    }
    // Display the Camera Extrinsic
	printf("Camera Extrinsic: \n");
    for (int i = 0; i < numberOfCamera; i++) {
		cout << cameraExtrinsic[i] << endl;
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