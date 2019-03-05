/*
 * Derived from
 * http://github.com/dusty-nv/jetson-inference
 */
 
#include <unistd.h>
#include <cmath>
#include <vector>
#include <random>
#include <assert.h>
#include <ros/ros.h>
#include "implay/playNet.h"

using namespace cv;
using namespace ros;

// stuff we know about the network and the caffe input/output blobs
static const int MAX_BATCH_SIZE = 2;
static const Scalar_<float> mean_vals(67.2, 103.3, 82.0);
static const Scalar_<float> scale_vals(1.0/255.0, 1.0/255.0, 1.0/255.0);

using namespace std;

int argmax(vector <double> &v)
{
    double cur_max = v[0];
    int cur_idx = 0;
    for (uint32_t i = 1; i < v.size(); i++) {
        if (v[i] > cur_max) {
            cur_max = v[i];
            cur_idx = i;
        }
    }
    return cur_idx;
}

void softmax(vector <double> &v, vector <double> &res)
{
    assert(res.size() == v.size());
    uint32_t len = v.size();
    double vmax = v[0];
    for (uint32_t i = 1; i < len; i++)
        if (v[i] > vmax)
            vmax = v[i];
    double sum = 0.0;
    for (uint32_t i = 0; i < len; i++) {
        double x = exp(v[i] - vmax);
        res[i] = x;
        sum += x;
    }
    for (uint32_t i = 0; i < len; i++) {
        res[i] /= sum;
    }
}

default_random_engine gen;
uniform_real_distribution<double> dist(0.0, 1.0);

int choice(vector <double> &v)
{
    double rand = dist(gen);

    double sum = 0.0;
    for (uint32_t i = 0; i < v.size(); i++) {
        sum += v[i];
        if (rand < sum) {
            return i;
        }
    }
    printf("Bogus distribution!\n");
    return 0;
}

// constructor
imNet::imNet() : tensorNet()
{
}

// destructor
imNet::~imNet()
{
}

imNet* imNet::Create(const char* prototxt_path, const char* model_path,
                           const char* input_layer_name, const char* output_layer_name)
{
    imNet* net = new imNet();
    
    if (!net)
        return NULL;
    
    if (!net->init(prototxt_path, model_path, input_layer_name, output_layer_name)) {
        printf("imNet -- failed to initialize.\n");
        return NULL;
    }

    return net;
}
    
bool imNet::init(const char* prototxt_path, const char* model_path,
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
    assert(mHeight == 128);
    assert(mWidth == 80);
    size_t step = mWidth * sizeof(float);
    size_t array_size = mHeight * step;
    uint8_t *p = (uint8_t *)mInputCUDA;
    for (int i = 0; i < 6; i++) {
        d_chans[i] = cv::cuda::GpuMat(mHeight, mWidth, CV_32FC1,
                     p + i * array_size, step);
    }
    // printf("setup: d_chans[0].data: %p\n", d_chans[0].data);

    printf("%s initialized.\n", model_path);
    return true;
}

bool imNet::preprocess(Mat &img1, Mat &img2)
{
    // Initial validation of the image
    if (img1.channels() + img2.channels() != 6) {
        printf("Input images must total 6 channels\n");
        exit(1);
    }
    if (img1.rows + img2.rows != 1024) {
        printf("Input image must have 512 rows\n");
        exit(1);
    }
    if (img1.cols + img2.cols != 640) {
        printf("Input image must have 320 columns\n");
        exit(1);
    }

    // Upload to gpu, resize, subract means and scale
    cv::cuda::GpuMat d_tmp;
    cv::cuda::GpuMat d_small;

    d_tmp.upload(img1);
    cuda::resize(d_tmp, d_small, Size(), 0.25, 0.25);
    cuda::cvtColor(d_small, d_small, CV_BGR2RGB, 3);
    d_small.convertTo(d_float1, CV_32FC3);
    cuda::subtract(d_float1, mean_vals, d_float1);
    cuda::multiply(d_float1, scale_vals, d_float1);

    d_tmp.upload(img2);
    cuda::resize(d_tmp, d_small, Size(), 0.25, 0.25);
    cuda::cvtColor(d_small, d_small, CV_BGR2RGB, 3);
    d_small.convertTo(d_float2, CV_32FC3);
    cuda::subtract(d_float2, mean_vals, d_float2);
    cuda::multiply(d_float2, scale_vals, d_float2);
    return true;
}

bool imNet::Predict(Mat &img1, Mat &img2, uint32_t* pres)
{
    // Time start = Time::now();
    if (!preprocess(img1, img2))
        return false;

    cuda::split(d_float1, &d_chans[0]);
    cuda::split(d_float2, &d_chans[3]);
    // Time pp = Time::now();
    // printf("after split: d_chans[0].data: %p\n", d_chans[0].data);

    // process with GIE
    void* inferenceBuffers[] = { mInputCUDA, mOutputs[0].CUDA };
    mContext->execute(1, inferenceBuffers);
    PROFILER_REPORT();

    float *f = &mOutputs[0].CPU[0];
    vector <double> score(f, f + 5);
    vector <double> sm(score.size());
    softmax(score, sm);
    // printf("implay %6.3f %6.3f %6.3f %6.3f %6.3f\n", sm[0], sm[1], sm[2], sm[3], sm[4]);
    int res = choice(sm);

    // double dp = (pp - start).toSec();
    // double de = (Time::now() - pp).toSec();
    // printf("pp: %5.3f, de: %5.3f\n", dp, de);

    *pres = res;
    
    return true;
}
