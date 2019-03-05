/*
 * Derived from
 * http://github.com/dusty-nv/jetson-inference
 */
 
#include <unistd.h>
#include <cmath>
#include <assert.h>
#include <ros/ros.h>

#include "score/scoreNet.h"

using namespace cv;
using namespace ros;

// stuff we know about the network and the caffe input/output blobs
static const int MAX_BATCH_SIZE = 2;
static const Scalar_<float> mean_vals(149.0, 178.0, 101.0);
static const Scalar_<float> scale_vals(1.0/255.0, 1.0/255.0, 1.0/255.0);

// Get gray value. 
float get_grv(Mat &image, int r, int c)
{
    float l;
    if (image.channels() == 3) {
        Vec3b pix = image.at<Vec3b>(r, c);
        float bl = (float)pix[0];
        float gr = (float)pix[1];
        float rd = (float)pix[2];
        l = gr * 0.587 + rd * 0.299 + bl * 0.114;
    } else {
        l = (float)image.at<uchar>(r, c);
    }
    return l;
}

int get_lit_pix(Mat &src, int h, int c)
{
    int r;

    for (r = 0; r < h; r++) {
        if (get_grv(src, r, c) > 50)
            break;
    }
    return r;
}

Point find_tl_corner(Mat &src, int w, int h)
{
    int r;
    int c;

    for (r = 0; r < h; r++)
        if (get_grv(src, r, 40) > 50)
            break;
    for (c = 0; c < w; c++)
        if (get_grv(src, 40, c) > 50)
            break;

    return Point(c, r);
}

uint32_t argmax(float *p, int n)
{
    int max_idx = 0;
    float max = p[0];
    for (int i = 1; i < n; i++) {
        if (p[i] > max) {
            max = p[i];
            max_idx = i;
        }
    }
    return max_idx;
}

// constructor
scoreNet::scoreNet() : tensorNet()
{
}

// destructor
scoreNet::~scoreNet()
{
}

scoreNet* scoreNet::Create(const char* prototxt_path, const char* model_path,
                           const char* input_layer_name, const char* output_layer_name)
{
    scoreNet* net = new scoreNet();
    
    if (!net)
        return NULL;
    
    if (!net->init(prototxt_path, model_path, input_layer_name, output_layer_name)) {
        printf("scoreNet -- failed to initialize.\n");
        return NULL;
    }

    return net;
}
    
bool scoreNet::init(const char* prototxt_path, const char* model_path,
                    const char* input_layer_name, const char* output_layer_name)
{
    /*
     * load and parse doonet network definition and model file
     */
    if (!tensorNet::LoadNetwork(prototxt_path, model_path, NULL,
        input_layer_name, output_layer_name)) {

        printf("failed to load %s\n", model_path);
        return false;
    }

    printf(LOG_GIE "%s loaded\n", model_path);

    // Wrap the GpuMats in d_chans around the input buffer
    assert(mHeight == 16);
    assert(mWidth == 68);
    size_t step = mWidth * sizeof(float);
    size_t array_size = mHeight * step;
    uint8_t *p = (uint8_t *)mInputCUDA;
    for (int i = 0; i < 3; i++) {
        d_chans[i] = cv::cuda::GpuMat(mHeight, mWidth, CV_32FC1,
                     p + i * array_size, step);
    }
    // printf("setup: d_chans[0].data: %p\n", d_chans[0].data);

    printf("%s initialized.\n", model_path);
    return true;
}

bool scoreNet::preprocess(Mat &src)
{
    const bool show_score = false;
    
    // Initial validation of the image
    if (src.channels() != 3) {
        printf("Input image must have 3 channels\n");
        exit(1);
    }
    if (src.rows != 512) {
        printf("Input image must have 512 rows\n");
        exit(1);
    }
    if (src.cols != 320) {
        printf("Input image must have 320 columns\n");
        exit(1);
    }

    // Extract the score
    const int r0 = 40;  // crop the score to 100x60
    const int c0 = 16;
    const int w = 120;
    const int h = 80;
    Point2f center(188.0, 230.0); // Phone's center of rotation
    const int c1 = 70; // used to measure the angle of the phone
    const int c2 = 250;
    const double dc = (float)(c2 - c1);
    Rect roi(c0, r0, w, h);

    // Crop general area and find the angle of the phone
    Mat score = src(roi);
    int r1 = get_lit_pix(src, src.rows, c1);
    int r2 = get_lit_pix(src, src.rows, c2);
    double angle = 180.0*asin((double)(r2 - r1)/dc)/M_PI; 

    // Derotate
    Mat rot = getRotationMatrix2D(center, angle, 1.0);
    Mat rotated_score;
    warpAffine(score, rotated_score, rot, Size(w, h));

    // Crop just the green portion from the derotated picture
    // mWidth and mHeight come from trained network, deploy.prototxt
    Point tl = find_tl_corner(rotated_score, w, h);
    tl.x += 12;
    tl.y += 6;
    if (tl.x + mWidth >= w || tl.y + mHeight >= h)
        return false;

    Point br(tl.x + mWidth, tl.y + mHeight);
    Mat cropped_score = rotated_score(Rect(tl, br));

    // Adjust contrast
    double minVal, maxVal;
    Mat flat_score = cropped_score.reshape(1);
    minMaxLoc(flat_score, &minVal, &maxVal); 
    float f = 255.0/maxVal;
    if (show_score) {
        Mat final_img = cropped_score * f;
        imshow("Score", final_img);
        waitKey(1);
    }
    // cvtColor(cropped_score, cropped_score, CV_BGR2RGB);
    cropped_score.convertTo(cropped_score, CV_32FC3);

    // Upload to gpu, convert colors, subract means and scale
    d_float.upload(cropped_score);
    cuda::multiply(d_float, Scalar_<float>(f, f, f), d_float);
    cuda::subtract(d_float, mean_vals, d_float);
    cuda::multiply(d_float, scale_vals, d_float);
    return true;
}

bool scoreNet::Predict(Mat &src, uint32_t* pres)
{
    // Time start = Time::now();
    if (!preprocess(src))
        return false;

    cuda::split(d_float, d_chans);
    // Time pp = Time::now();
    // printf("after split: d_chans[0].data: %p\n", d_chans[0].data);

    // process with GIE
    void* inferenceBuffers[] = { mInputCUDA, mOutputs[0].CUDA };
    mContext->execute(1, inferenceBuffers);
    PROFILER_REPORT();

    int res = 0;
    for( size_t n=0; n < 5; n++ ) {
        uint32_t dig = argmax(&mOutputs[0].CPU[n*11], 11);
        if (dig == 10)
            break;
        res = res * 10 + dig;
    }

    // double dp = (pp - start).toSec();
    // double de = (Time::now() - pp).toSec();
    // printf("pp: %5.3f, de: %5.3f\n", dp, de);

    *pres = res;
    
    return true;
}
