/*
 * Loads a single image and displays it
 */
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;
using namespace std;
using namespace sensor_msgs;

ImageConstPtr smp = NULL;
bool save_frames = false;
char *frame_dir;
uint32_t frame_number = 0;
uint32_t last_seq = 0;
void image_callback(const ImageConstPtr &mp)
{
    smp = mp;
}

void handle_msg()
{
    Mat img = Mat((uint32_t)smp->height, (uint32_t)smp->width, CV_8UC3,
                  (uint8_t *)&smp->data[0], (uint32_t)smp->step);
    if (img.rows == 0 || img.rows == 0 || img.data == NULL) {
        printf("viz bad frame\n");
	smp = NULL;
        return;
    }
    if (smp->header.seq != ++last_seq) {
        printf("skipped %d\n", last_seq);
        last_seq = smp->header.seq;
    }
    if (save_frames) {
        frame_number++;
        char file_name[50];
        sprintf(file_name, "%s/%04d.png", frame_dir, frame_number);
        imwrite(file_name, img);
    }
    Point tl(30, 67);
    Point br(295, 456);
    Rect r(tl, br);
    rectangle(img, r, Scalar(255, 255, 255));
    imshow("image", img);
    int key = waitKey(10);
    key &= 0xff;
    if (key == 'q')
        exit(1);
    smp = NULL;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "infer");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("camera1/image_raw", 1, image_callback);

    if (strcmp(argv[1], "-s") == 0) {
        save_frames = true;
        frame_dir = argv[2];
    }
    while (ros::ok()) {

        ros::spinOnce();

        if (smp != NULL)
            handle_msg();

    }
    printf("viz: shutting down\n");
    return(0);
}
