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
    if(allocationList_) { delete(allocationList_); }
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
    bool doIntegrate = checkPoseKernel(pose_, old_pose_,
                                       reduction_output_.data(),
                                       computation_size_, track_threshold);

    if ((doIntegrate && ((frame % integration_rate) == 0)) || (frame <= 3)) {

        float voxelsize =  volume_._dim/volume_._size;
        int num_vox_per_pix = volume_._dim/((se::VoxelBlock<FieldType>::side)*voxelsize);
        size_t total = num_vox_per_pix * computation_size_.x() * computation_size_.y();
        allocation_list_.reserve(total);

        unsigned int allocated = 0;
        if(std::is_same<FieldType, SDF>::value) {
            allocated  = buildAllocationList(allocation_list_.data(),
                                             allocation_list_.capacity(),
                                             *volume_._map_index, pose_, getCameraMatrix(k), float_depth_.data(),
                                             computation_size_, volume_._size,
                                             voxelsize, 2*mu);
        } else if(std::is_same<FieldType, OFusion>::value) {
            allocated = buildOctantList(allocation_list_.data(), allocation_list_.capacity(),
                                        *volume_._map_index,
                                        pose_, getCameraMatrix(k), float_depth_.data(), computation_size_, voxelsize,
                                        compute_stepsize, step_to_depth, 6*mu);
        }

        volume_._map_index->allocate(allocation_list_.data(), allocated);

        if(std::is_same<FieldType, SDF>::value) {
            if (render_color_) {
                struct sdf_update funct(float_depth_.data(), float_rgb_.data(),
                                        Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu, 100);
                se::functor::projective_map(*volume_._map_index,
                                            Sophus::SE3f(pose_).inverse(),
                                            getCameraMatrix(k),
                                            Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
                                            funct);
            } else {
                struct sdf_update funct(float_depth_.data(),
                                        Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu, 100);
                se::functor::projective_map(*volume_._map_index,
                                            Sophus::SE3f(pose_).inverse(),
                                            getCameraMatrix(k),
                                            Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
                                            funct);
            }

        } else if(std::is_same<FieldType, OFusion>::value) {

            float timestamp = (1.f/30.f)*frame;
            struct bfusion_update funct(float_depth_.data(),
                                        Eigen::Vector2i(computation_size_.x(), computation_size_.y()), mu, timestamp);

            se::functor::projective_map(*volume_._map_index,
                                        Sophus::SE3f(pose_).inverse(),
                                        getCameraMatrix(k),
                                        Eigen::Vector2i(computation_size_.x(), computation_size_.y()),
                                        funct);
        }

        doIntegrate = true;
    } else {
        doIntegrate = false;
    }

    return doIntegrate;

}
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

