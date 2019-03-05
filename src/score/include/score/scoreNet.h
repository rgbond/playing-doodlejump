/*
 * derived from 
 * http://github.com/dusty-nv/jetson-inference
 */
 
#ifndef __LASER_NET_H__
#define __LASER_NET_H__

#include <vector>

#include "tensorNet.h"

#include "opencv2/opencv.hpp"
#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

/**
 * Doodle Jump scores
 */
class scoreNet : public tensorNet
{
public:

    /**
     * Load a new network instance
     */
     static scoreNet* Create(const char* prototxt_path, const char* model_path,
                             const char* input_layer_name="data",
                             const char* output_layer_name="score");
    
    /**
     * Destroy
     */
    virtual ~scoreNet();
    
    /**
     * Predict the score
     */
    bool Predict(cv::Mat &src, uint32_t* result );

private:
    scoreNet();
    
    bool init(const char* prototxt_path, const char* model_path, 
              const char* input_layer_name, const char* output_layer_name);
    bool preprocess(cv::Mat &src);
    
    cv::cuda::GpuMat d_float;
    cv::cuda::GpuMat d_chans[3];
};

#endif
