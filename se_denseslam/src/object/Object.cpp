//
// Created by dorianhenning on 09/11/18.
//

#include "se/object/Object.h"

#include "../bfusion/mapping_impl.hpp"
#include "../kfusion/mapping_impl.hpp"
#include "../bfusion/alloc_impl.hpp"
#include "../kfusion/alloc_impl.hpp"


Object::Object(const int                voxelBlockSize,
               const Eigen::Vector3f&   volumeDimensions,
               const Eigen::Vector3i&   volumeResolution,
               const Eigen::Matrix4f&   pose,
               const int                classID,
               const ProbVector&        probVector,
//               const Eigen::Matrix<float, 1, 80>&        probVector,
               const Eigen::Vector2i    imgSize)
    : voxelBlockSize_(voxelBlockSize),
      volumeDimensions_(volumeDimensions),
      volumeResolution_(volumeResolution),
      volumePose_(pose),
      vertex_(imgSize(0), imgSize(1)),
      normal_(imgSize(0), imgSize(1)),
      classID_(classID),
      semanticFusion_(probVector)
{
    // TODO: implement this
//    volume_(volumeResolution_(0), volumeDimensions_(0));
    discrete_vol_ptr_ = std::make_shared<se::Octree<FieldType> >();
    discrete_vol_ptr_->init(volumeResolution_(0), volumeDimensions(0));
    volume_ = Volume<FieldType>(volumeResolution_(0), volumeDimensions(0),
                                discrete_vol_ptr_.get());

    fusedTime_ = 1;
}

Object::~Object()
{
//    this->volume_.release();
    if(allocationList_) { delete(allocationList_); }
//    free(vertex_);
//    free(normal_);
}

void Object::integrateBackgroundKernel(const Eigen::Vector4f    k,
                                       const float              mu,
                                       const uint               frame,
                                       const Eigen::Matrix4f&   T_w_c,
                                       const float *            float_depth,
                                       const Eigen::Vector3f *  float_rgb,
                                       const Eigen::Vector2i    frameSize)
{
    // TODO: implement this
}

void Object::integrateVolumeKernel()
{
    // TODO: implement this
}

void Object::fuseSemanticKernel()
{
    // TODO: implement this
}

void Object::fuseSemanticLabel()
{
    // TODO: implement this
}

