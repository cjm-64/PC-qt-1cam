#ifndef GLOBALS_H
#define GLOBALS_H


class globals
{
public:
    globals();
};
//Timing
extern int Test_Time;
extern int Test_Time_Frames;
extern int Test_Time_M;
extern int Test_Time_H;
extern int SetUpTime;
extern int CalibrationTime;
extern int TimeCount;
extern int RecordingTimer;
extern int TestCompleteFlag;

//Box Deprecated
extern int X;
extern int Y;
extern int Width;
extern int Height;

//Both Circles
extern int thresh_max_val;
extern int thresh_type;
extern int CED;
extern int Cent_D;

//Right Circle
extern int R_thresh_val;
extern int R_max_rad;

//Left Circle
extern int L_thresh_val;
extern int L_max_rad;

//Misc
extern int Step;
extern int OnOff;
extern int DisplaySelector;
extern int DataSavingFlag;
extern int ColorOrBW;


#endif // GLOBALS_H
