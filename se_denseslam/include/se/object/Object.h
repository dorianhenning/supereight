//
// Created by dorianhenning on 09/11/18.
//

#ifndef SUPEREIGHT_OBJECT_H
#define SUPEREIGHT_OBJECT_H

#include <se/commons.h>
#include <se/image/image.hpp>
#include <se/utils/math_utils.h>

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

    Object(const int                voxelBlockSize,
           const Eigen::Vector3f&   volumeDimensions,
           const Eigen::Vector3i&   volumeResolution,
           const Eigen::Matrix4f&   pose,
           const int                classID,
           const ProbVector&        probVector,
//           const Eigen::Matrix<float, 1, 80>&        probVector,
           const Eigen::Vector2i    imgSize);

    inline void setVolumeDimensions(const Eigen::Vector3f volumeDimensions)
    {
        this->volumeDimensions_ = volumeDimensions;
    };

    inline int getVoxelBlockSize() const
    {
        return this->voxelBlockSize_;
    }

    inline Eigen::Vector3f getVolumeDimensions() const
    {
        return this->volumeDimensions_;
    };

    inline Eigen::Vector3i getVolumeResolution() const
    {
        return this->volumeResolution_;
    };

    void integrateBackgroundKernel(const Eigen::Vector4f    k,
                                   const float              mu,
                                   const unsigned int       frame,
                                   const Eigen::Matrix4f&   T_w_c,
                                   const float *            float_depth,
                                   const Eigen::Vector3f *  float_rgb,
                                   const Eigen::Vector2i    frameSize);

    void integrateVolumeKernel();

    void fuseSemanticKernel();

    void fuseSemanticLabel();

    virtual ~Object();

private:
    // Volume
    unsigned int voxelBlockSize_ = 8;
    Eigen::Vector3f volumeDimensions_;
    Eigen::Vector3i volumeResolution_;
//    float volumeStep = min(volumeSize_) / max(volumeResolution_);

    // inter-frame
    se::Image<Eigen::Vector3f> vertex_;
    se::Image<Eigen::Vector3f> normal_;

    std::vector<se::key_t> * allocationList_;
    std::shared_ptr<se::Octree<FieldType> > discrete_vol_ptr_;
    Volume<FieldType> volume_;
    size_t reserved_  = 0;

    // labels
    int classID_;
    int instanceLabel_;

    // pose
    Eigen::Matrix4f volumePose_;

    // semantics
    ProbVector semanticFusion_ = ProbVector::Zero();
//    Eigen::Matrix<float, 1, 80> semanticFusion_ = Eigen::Matrix<float, 1, 80>::Zero();
    const size_t labelSize = 80;
    int fusedTime_;

};

#endif //SUPEREIGHT_OBJECT_H
