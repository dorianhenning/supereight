//
// Created by dorianhenning on 09/11/18.
//

#ifndef SUPEREIGHT_OBJECTMAPPINGSYSTEM_H
#define SUPEREIGHT_OBJECTMAPPINGSYSTEM_H

#include <cstdlib>
#include <se/commons.h>
#include <iostream>
#include <memory>
#include <perfstats.h>
#include <timings.h>
#include <se/config.h>
#include <se/octree.hpp>
#include <se/image/image.hpp>
#include "volume_traits.hpp"
#include "continuous/volume_template.hpp"

/*
 * Use SE_FIELD_TYPE macro to define the ObjectMappingSystem instance
 */
typedef SE_FIELD_TYPE FieldType;
template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

class ObjectMappingSystem {

private:
    uint2 computation_size_;
    Matrix4 pose_;
    Matrix4 *viewPose_;
    float3 volume_dimension_;
    uint3 volume_resolution_;
    std::vector<int> iterations_;
    bool tracked_;
    bool integrated_;
    float3 init_pose_;
    float mu_;
    bool need_render_ = false;
    Configuration config_;

    // color integration
    bool render_color_ = true;

    // segmentation
    Object * objectList = new Object();

    // input once
    std::vector<float> gaussian_;

    // inter-frame
    se::Image<Eigen::Vector3f> vertex_;
    se::Image<Eigen::Vector3f> normal_;

    std::vector<se::key_t> allocation_list_;
    std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
    Volume<FieldType> volume_;

    // intra-frame
    std::vector<float> reduction_output_;
    std::vector<se::Image<float>  > scaled_depth_;
    std::vector<se::Image<Eigen::Vector3f> > input_vertex_;
    std::vector<se::Image<Eigen::Vector3f> > input_normal_;
    se::Image<float> float_depth_;
    se::Image<Eigen::Vector3f> float_rgb_;
    se::Image<float> float_grey_; // for intensity of color
//    se::Image<TrackData>  tracking_result_;
    std::vector<TrackData>  tracking_result_;
    Matrix4 old_pose_;
    Matrix4 raycast_pose_;
};

#endif //SUPEREIGHT_OBJECTMAPPINGSYSTEM_H
