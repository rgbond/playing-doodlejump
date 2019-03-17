/*
 * Loads a single image and runs the gie setup on it
 * Only argument is path to the image
 */
#include <sys/time.h>
#include <stdlib.h>
#include <stdio.h>
#include <queue>
#include <vector>

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <score/DoodleScore.h>

#include "score/scoreNet.h"

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
        printf("Score dropping a message\n");
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

void send_msg(ImageConstPtr mp, int32_t result, ros::Publisher &pub)
{
    score::DoodleScore dscore;
    dscore.fseq = mp->header.seq;
    dscore.score = result;
    pub.publish(dscore);
}

void send_test_message(ros::Publisher &pub)
{
    static int  seq;

    score::DoodleScore dscore;
    dscore.fseq = seq++;
    dscore.score = 0;
    printf("send_test_message %d %d\n", dscore.fseq, dscore.score);
    pub.publish(dscore);
}

void handle_msgs(scoreNet *net, ros::Publisher &pub)
{
    uint32_t result;
    Mat m1;
    ImageConstPtr mp0 = cbq;
    switch(nmsgs) {
        case 1: 
            m1 = decode_msg(mp0);
            // imshow("ScoreFrame", m1);
            // waitKey(10);
            if (net->Predict(m1, &result)) {
                send_msg(mp0, result, pub);
            } else {
                printf("score: predict failed %d\n", mp0->header.seq);
            }
            break;
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
    ros::Publisher pub = nh.advertise<score::DoodleScore>("score/DoodleScore", 20);

    if (argc == 2 && string(argv[1]) == "-t") {
        ros::Rate r(30);
        while (ros::ok()) {
            send_test_message(pub);
            r.sleep();
        }
        return(0);
    }

    ros::Subscriber sub = nh.subscribe("camera1/image_raw", 2, image_callback);

    const char *deploy = "/caffe/ros/src/score/src/model/net10_deploy.prototxt";
    const char *model =  "/caffe/ros/src/score/src/model/net10_ft1_1908000.caffemodel";
    scoreNet *net = scoreNet::Create(deploy, model);

    while (ros::ok()) {

        ros::spinOnce();

        if (nmsgs != 0)
            handle_msgs(net, pub);
    }
    printf("score: shutting down");
    return(0);
}
