#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "globals.h"

#include "stdio.h"
#include "iostream"
#include "stdio.h"
#include "unistd.h"
#include "string"
#include "libuvc/libuvc.h"
#include "turbojpeg.h"
#include "math.h"
#include "chrono"
#include "list"
#include "fstream"

// Computer Vision
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;


//Data strucures
struct CamSettings{
    string CamName;
    int width;
    int height;
    int fps;
};

// ID0 = right, ID1 = Left, ID2 = World
struct CamInfo{
    string CamName;
    uint8_t vID;
    uint8_t pID;
    string uid;
    int cam_num;
};

struct StreamingInfo{
    uvc_context_t *ctx;
    uvc_device_t *dev;
    uvc_device_handle_t *devh;
    uvc_stream_ctrl_t ctrl;
    uvc_stream_handle_t *strmh;
    const uvc_format_desc_t *format_desc;
    uvc_frame_desc_t *frame_desc;
    enum uvc_frame_format frame_format;
};

struct FrameProcessingInfo{
    int thresh_val; //Threshold value for gray to binary conversion
    int max_radius; //Max radius circle to search for in HoughCircles()
};

struct PositionData{
    int X_Pos;
    int Y_Pos;
    int Radius;
};

//Instantiate global data strucutres
CamSettings camSets[2];
CamInfo Cameras[2];
StreamingInfo CamStreams[2];
FrameProcessingInfo FrameProc[2];

//Other global declarations
Scalar col = Scalar(0, 255, 0); //Color for drawing on frame

//Functions to set up cameras
void getCamInfo(struct CamInfo *ci){
    uvc_context_t *ctx;
    uvc_device_t **device_list;
    uvc_device_t *dev;
    uvc_device_descriptor_t *desc;
    uvc_error_t res;

    res = uvc_init(&ctx,NULL);
    if(res < 0) {
        uvc_perror(res, "uvc_init");
    }

    res = uvc_get_device_list(ctx,&device_list);
    if (res < 0) {
        uvc_perror(res, "uvc_find_device");
    }
    int i = 0;
    int dev_struct_count = 0;
    while (true) {
        cout << i << endl;
        dev = device_list[i];
        if (dev == NULL) {
            break;
        }
        else{
            uvc_get_device_descriptor(dev, &desc);
            if(desc->idVendor != 3141){
                printf("World Cam\n");
                i++;
                continue;
            }
            else{
                ci[dev_struct_count].CamName = desc->product;
                ci[dev_struct_count].vID = desc->idVendor;
                ci[dev_struct_count].pID = desc->idProduct;
                ci[dev_struct_count].uid = to_string(uvc_get_device_address(dev))+":"+to_string(uvc_get_bus_number(dev));
                ci[dev_struct_count].cam_num = i;
                dev_struct_count++;
            }
            printf("Got desc\n");
        }
        uvc_free_device_descriptor(desc);
        i++;
    }
    uvc_exit(ctx);
}

void setUpStreams(struct CamSettings *cs, struct CamInfo *ci, struct StreamingInfo *si){
    uvc_error_t res;
    uvc_device_t **devicelist;


    res = uvc_init(&si[0].ctx, NULL);
    if (res < 0) {
        uvc_perror(res, "uvc_init");
    }
    else{
        printf("UVC %d open success\n", 0);
    }
    res = uvc_find_devices(si[0].ctx, &devicelist, 0, 0, NULL);
    if (res < 0) {
        uvc_perror(res, "uvc_init");
    }
    else{
        cout << "Dev " << 0 << ": " << si[0].dev << endl;
    }
    res = uvc_open(devicelist[ci[0].cam_num], &si[0].devh, 1);
    if (res < 0) {
        uvc_perror(res, "uvc_find_device"); /* no devices found */
    }
    else{
        cout << "devh " << 0 << ": " << si[0].devh << endl;
    }
    si[0].format_desc = uvc_get_format_descs(si[0].devh);
    si[0].frame_desc = si[0].format_desc->frame_descs->next;
    si[0].frame_format = UVC_FRAME_FORMAT_ANY;
    if(si[0].frame_desc->wWidth != NULL){
        cs[0].width = si[0].frame_desc->wWidth;
        cs[0].height = si[0].frame_desc->wHeight;
        cs[0].fps = 10000000 / si[0].frame_desc->intervals[2];
    }
    printf("\nEye %d format: (%4s) %dx%d %dfps\n\n", 0, si[0].format_desc->fourccFormat, cs[0].width, cs[0].height, cs[0].fps);

    res = uvc_get_stream_ctrl_format_size(si[0].devh, &si[0].ctrl, si[0].frame_format, cs[0].width, cs[0].height, cs[0].fps, 1);
    if (res < 0){
        uvc_perror(res, "start_streaming");
    }
    else{
        printf("Eye %d stream control formatted\n", 0);
        uvc_print_stream_ctrl(&si[0].ctrl, stderr);
    }

    res = uvc_stream_open_ctrl(si[0].devh, &si[0].strmh, &si[0].ctrl,1);
    if (res < 0){
        uvc_perror(res, "start_streaming");
    }
    else{
        printf("Eye %d stream opened\n", 0);
    }

    res = uvc_stream_start(si[0].strmh, nullptr, nullptr,2.0,0);
    if (res < 0){
        uvc_perror(res, "start_streaming");
    }
    else{
        printf("Eye %d stream started\n", 0);
    }
    printf("\n\n\n");
}

void MainWindow::initFrameProc(){
    //Set the inital values for the frame proc struct
    for (int i=0; i<2; i++){
        FrameProc[i].thresh_val = 50;
        FrameProc[i].max_radius = 50;
    }

}

void MainWindow::initCams(){
    for(int i = 0; i<2;i++){
        if(i == 0){
            camSets[i].CamName = "Pupil Cam2 ID0";
        }
        else{
            camSets[i].CamName = "Pupil Cam2 ID1";
        }
        camSets[i].width = 192;
        camSets[i].height = 192;
        camSets[i].fps = 120;
    }

    getCamInfo(Cameras);
    for (int j = 0; j<2; j++){
        cout << "Cam " << j << ": " << Cameras[j].CamName << " " << Cameras[j].uid << " " << Cameras[j].vID << " " << Cameras[j].pID << endl;
    }

    setUpStreams(camSets, Cameras, CamStreams);

}

void MainWindow::openCamera(){
    qDebug() << "Open Stream";
    connect(timer, SIGNAL(timeout()), this, SLOT(updateFrame()));
    qDebug() << "Slot Connected";
    timer->start(20);
    qDebug() << "Timer Started";
}

void MainWindow::updateFrame(){

    qDebug() << "UF";

    uvc_frame_t *frame;
    uvc_frame_t *bgr;
    uvc_error_t res;

    Mat image;
    //As a side note, the images are greyscaled, however they are treated as color when they come into the code

    res = uvc_stream_get_frame(CamStreams[0].strmh, &frame, 1* pow(10,6));
    if(res < 0){
        uvc_perror(res, "Failed to get frame");
    }
    else{
//        printf("got frame");
    }

    //Allocate buffers for conversions
    int frameW = frame->width;
    int frameH = frame->height;
    long unsigned int frameBytes = frame->data_bytes;

//    printf("Eye %d: frame_format = %d, width = %d, height = %d, length = %lu\n", 0, frame->frame_format, frameW, frameH, frameBytes);

    bgr = uvc_allocate_frame(frameW * frameH * 3);
    if (!bgr) {
        printf("unable to allocate bgr frame!\n");
        return;
    }
    res = uvc_yuyv2bgr(frame, bgr);
    if (res < 0){
        printf("Unable to copy frame to bgr!\n");
    }
    Mat placeholder(bgr->height, bgr->width, CV_8UC3, bgr->data); //Create MAT and fill with frame data
    Mat flipped;
    placeholder.copyTo(flipped); //Copy frame to new MAT. idk why you need to do this but you do ¯\_(ツ)_/¯

    flip(flipped, image, 0); //Flip image

    Mat grayIMG, binaryIMG, bpcIMG; //Create new Mats to to image processing steps
    cvtColor(image, grayIMG, COLOR_BGR2GRAY); //Convert to grayscale
    threshold(grayIMG, binaryIMG, FrameProc[0].thresh_val, thresh_max_val, thresh_type); //Convert to binary based on thresh; controlled by slider
    cvtColor(binaryIMG, bpcIMG, COLOR_GRAY2RGB); // enable color on binary so we can draw on it later

    PositionData pd;
    vector<Vec3f> circles;
    HoughCircles(binaryIMG, circles, HOUGH_GRADIENT, 1, 1000, CED, Cent_D, FrameProc[0].max_radius-2, FrameProc[0].max_radius);
    Vec3i c;
    for( size_t i = 0; i < circles.size(); i++ ){
        c = circles[i];
    }

    pd.X_Pos = c[0];
    pd.Y_Pos = c[1];
    pd.Radius = c[2];
    cout << "X , Y, Rad: " << pd.X_Pos << ", " << pd.Y_Pos << ", " << pd.Radius << endl;

    //savePositions(pd);
    //Draw Circles on Black and White
    circle(bpcIMG, Point(pd.X_Pos, pd.Y_Pos), 1, col,1,LINE_8);
    circle(bpcIMG, Point(pd.X_Pos, pd.Y_Pos), pd.Radius, col,1,LINE_8);

    Mat final_image;
    //Display Image
    //Check for which eye and if grey or Black and White (binary)


    bpcIMG.copyTo(final_image);
    ui->Display->setPixmap(QPixmap::fromImage(QImage((unsigned char*) final_image.data, final_image.cols, final_image.rows, final_image.step, QImage::Format_RGB888)));


    //Free memory
    flipped.release();
    image.release();
    placeholder.release();
    grayIMG.release();
    binaryIMG.release();
    bpcIMG.release();
    uvc_free_frame(bgr);
}


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    ui->horizontalLayoutWidget->setVisible(false);

    timer = new QTimer(this);

    initCams();
    initFrameProc();
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::on_pushButton_clicked()
{
    ui->horizontalLayoutWidget->setVisible(true);
    openCamera();
}


void MainWindow::on_RadiusSlider_valueChanged(int Radius)
{
    ui->RadiusDisplay->display(Radius);
    FrameProc[0].max_radius = Radius;
}


void MainWindow::on_ThresholdSlider_valueChanged(int Threshold)
{
    ui->ThresholdDisplay->display(Threshold);
    FrameProc[0].thresh_val = Threshold;
}


void MainWindow::on_CloseCameras_clicked()
{
    timer->stop();
    uvc_exit(CamStreams[0].ctx);
}

