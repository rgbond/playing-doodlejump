/*
 * derived from 
 * http://github.com/dusty-nv/jetson-inference
 */
 
#ifndef __LASER_NET_H__
#define __LASER_NET_H__

#include <vector>

#include "tensorNet.h"

#include <opencv2/cudawarping.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudaarithm.hpp>

/**
 * Play imitation learing Doodle Jump
 */
class imNet : public tensorNet
{
public:

    /**
     * Load a new network instance
     */
     static imNet* Create(const char* prototxt_path, const char* model_path,
                             const char* input_layer_name="data",
                             const char* output_layer_name="score");
    
    /**
     * Destroy
     */
    virtual ~imNet();
    
    /**
     * Predict an action
     */
    bool Predict(cv::Mat &img1, cv::Mat &img2, uint32_t* result );

private:
    imNet();
    
    bool init(const char* prototxt_path, const char* model_path, 
              const char* input_layer_name, const char* output_layer_name);
    bool preprocess(cv::Mat &src1, cv::Mat &src2);
    
    cv::cuda::GpuMat d_float1;
    cv::cuda::GpuMat d_float2;
    cv::cuda::GpuMat d_chans[6];
};

#endif
