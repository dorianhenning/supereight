//
// Created by dorianhenning on 09/11/18.
//

#ifndef SUPEREIGHT_OBJECT_H
#define SUPEREIGHT_OBJECT_H

#include "math_utils.h"

/*
 * Use SE_FIELD_TYPE macro to define the DenseSLAMSystem instance.
 */
typedef SE_FIELD_TYPE FieldType;
template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

class Object;
typedef std::shared_ptr<Object> ObjectPointer;
typedef std::vector<ObjectPointer> ObjectList;
typedef ObjectList::iterator ObjectListIterator;

class Object {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Object(const int              voxelBlockSize,
           const Eigen::Vector3f& volumeSize,
           const Eigen::Vector3i& volumeResolution,
           const Eigen::Matrix4f& pose,
           const int              classID,
           const ProbVector&      probVector,
           const Eigen::Vector2i  imgSize);

    inline void setVolumeSize(const Eigen::Vector3f volumeSize)
    {
        this->volumeSize_ = volumeSize;
    };

    inline int getVoxelBlockSize() const
    {
        return this->voxelBlockSize_;
    }

    inline Eigen::Vector3f getVolumeSize() const
    {
        return this->volumeSize_;
    };

    inline Eigen::Vector2i getVolumeResolution() const
    {
        return this->volumeResolution_;
    };

    void integrateBackgroundKernel();

    void integrateVolumeKernel();

    void fuseSemanticKernel();

    void fuseSemanticLabel();

private:
    // Volume
    unsigned int voxelBlockSize_ = 8;
    Eigen::Vector3f volumeSize_;
    Eigen::Vector2i volumeResolution_;
//    float volumeStep = min(volumeSize_) / max(volumeResolution_);

    // inter-frame
    se::Image<Eigen::Vector3f> vertex_;
    se::Image<Eigen::Vector3f> normal_;

    std::vector<se::key_t> allocationList_;
//    std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
    Volume<FieldType> volume_;
    size_t reserved_  = 0;

    // labels
    int classID_;
    int instanceLabel_;

    // pose
    Eigen::Matrix4f volumePose_;

    // semantics
    ProbVector semanticFusion_  = ProbVector::Zero();
    const size_t labelSize = 80;

};

#endif //SUPEREIGHT_OBJECT_H
