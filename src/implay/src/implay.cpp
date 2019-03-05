/*
 * Loads a single image and runs the gie setup on it
 * Only argument is path to the image
 */
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <implay/DoobotActionArchive.h>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "implay/playNet.h"

using namespace cv;
using namespace std;
using namespace sensor_msgs;
using namespace ros;

// Fix me
string correct_encoding = string(image_encodings::BGR8);
int nmsgs = 0;
ImageConstPtr cbq;

void image_callback(const ImageConstPtr &mp)
{
    if (nmsgs == 1) {
        printf("Implay dropping a message\n");
        return;
    }
    cbq = mp;
    nmsgs++;
}

Mat decode_msg(ImageConstPtr mp)
{
    string msg_encoding = string(mp->encoding);
    if (msg_encoding != correct_encoding) {
        printf("Score bad encoding\n");
        exit(1);
    }
    return Mat((uint32_t)mp->height, (uint32_t)mp->width, CV_8UC3,
               (uint8_t *)&mp->data[0], (uint32_t)mp->step);
}

void send_msgs(ImageConstPtr mp, int32_t result, ros::Publisher &archive_pub)
{
    // printf("implay send_msgs %d\n", result);
    // Tell archiver what to do
    implay::DoobotActionArchive aaction;
    aaction.action = result;
    aaction.fseq = mp->header.seq;
    archive_pub.publish(aaction);
}

void send_test_messages(ros::Publisher &archive_pub)
{
    static int  seq;

    implay::DoobotActionArchive aaction;
    aaction.action = 0;
    aaction.fseq = seq++;
    archive_pub.publish(aaction);
}

const int skip = 3;

void handle_msgs(imNet *net, ros::Publisher &archive_pub)
{
    static Mat last_img;
    static bool have_last_img = false;

    uint32_t result;
    Mat new_img;
    ImageConstPtr mp = cbq;
    switch(nmsgs) {
        case 1: 
        {
            if (mp->header.seq % skip != 0)
                break;
            new_img = decode_msg(mp);
            if (have_last_img) {
                if (net->Predict(last_img, new_img, &result)) {
                    if (result != 0)
                        send_msgs(mp, result, archive_pub);
                } else {
                    printf("implay: predict failed on frame %d\n", mp->header.seq);
                }
            } else {
                have_last_img = true;
            }
            last_img = new_img;
            break;
        }
        default:
            printf("Why are we here? nmsgs = %d/n", nmsgs);
            break;
    }
    nmsgs = 0;
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "infer");
    ros::NodeHandle nh;
    ros::Publisher archive_pub = 
            nh.advertise<implay::DoobotActionArchive>("implay/DoobotActionArchive", 20);

    if (argc == 2 && string(argv[1]) == "-t") {
        ros::Rate r(30);
        while (ros::ok()) {
            send_test_messages(archive_pub);
            r.sleep();
        }
        return(0);
    }

    ros::Subscriber sub = nh.subscribe("camera1/image_raw", 2, image_callback);

    const char *deploy = "/caffe/doodle/imnet/imitate/deploy.prototxt";
    const char *model =  "/caffe/doodle/imnet/snapshot/keep/net1j_200000.caffemodel";
    printf("implay using %s\n", model);
    imNet *net = imNet::Create(deploy, model);

    while (ros::ok()) {

        ros::spinOnce();

        if (nmsgs != 0)
            handle_msgs(net, archive_pub);
    }
    printf("implay: shutting down");
    return(0);
}
