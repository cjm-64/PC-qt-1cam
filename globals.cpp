#include "globals.h"

//Test Timing
int Test_Time = 0; //Total time in seconds
int Test_Time_Frames = 0; //Time in frames (seconds * 30)
int Test_Time_M = 1; //Minutes component of test time
int Test_Time_H = 0; //Hours components of test time
int SetUpTime = 300000; //5 minutes to set up tacking
int CalibrationTime = 3000; //3 seconds of calibration (3*30fps = 90)
int TimeCount = 0; //Frame counter for timing
int RecordingTimer = 0; //Flag to set how long to record for
int TestCompleteFlag = 0; //Flag for if the test was completed

//Box Deprecated
int X = 320;
int Y = 240;
int Width = 160;
int Height = 120;

//Both Circles
int thresh_max_val = 100; //Max threshold value
int thresh_type = 1; //Type of threshold, read OCV documentation
int CED = 1; //For Circle tracking, read OCV documentation
int Cent_D = 1; //For Circle tracking, read OCV documentation

//Right Circle
int R_thresh_val = 50; //Threshold Value
int R_max_rad = 50; //Radius value


//Left Circle
int L_thresh_val = 50; //Threshold Value
int L_max_rad = 50; //Radius value

//Misc
int Step = 0; //Define the step
int DisplaySelector = 0; //Select which display to use; 0 = w/Panel 1 = w/o
int DataSavingFlag = 0; //Where to save the data; 0 = None, 1 = CalR, 2 = CalL 3 = Full Test
int ColorOrBW = 0; //Color Video or BW Video; Color = 0, BW = 1


//Tracking
int CalRFrame = 0;
int CalLFrame = 0;
int TestFrame = 0;
int RightCalibrationBefore[200] = {0}; //Array for saving the calibration data before the test
int RightCalibrationAfter[200] = {0}; //Array for saving the calibration data before the test
int LeftCalibrationBefore[200] = {0}; //Array for saving the calibration data before the test
int LeftCalibrationAfter[200] = {0}; //Array for saving the calibration data before the test
int XPositionServer[558000] = {0}; //Array for saving the calibration data during the test
int XPositionClient[558000] = {0}; //Deprecated
int CalFrames = 0;
int X_Point = 0; //X of center of tracked circle (pupil)
int Y_Point = 0; //Y of center of tracked circle (pupil)
int Radius = 0; //Radius of tracked circle (pupil)

globals::globals()
{

}
