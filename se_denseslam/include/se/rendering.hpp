/*

 Copyright (c) 2014 University of Edinburgh, Imperial College, University of Manchester.
 Developed in the PAMELA project, EPSRC Programme Grant EP/K008730/1

 This code is licensed under the MIT License.


 Copyright 2016 Emanuele Vespa, Imperial College London

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
 list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 3. Neither the name of the copyright holder nor the names of its contributors
 may be used to endorse or promote products derived from this software without
 specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef __RENDERING_HPP
#define __RENDERING_HPP

#include <tuple>
#include <sophus/se3.hpp>

#include <timings.h>
#include <se/utils/math_utils.h>
#include <se/commons.h>
#include <se/continuous/volume_template.hpp>
#include <se/image/image.hpp>
#include <se/ray_iterator.hpp>

/* Raycasting implementations */
#include <se/bfusion/rendering_impl.hpp>
#include <se/kfusion/rendering_impl.hpp>

template <typename T>
using Volume = VolumeTemplate<T, se::Octree>;

template<typename T>
void raycastKernel(const Volume<T>& volume, se::Image<Eigen::Vector3f>& vertex,
   se::Image<Eigen::Vector3f>& normal,
   const Eigen::Matrix4f& view, const float nearPlane, const float farPlane,
   const float mu, const float step, const float largestep) {
  TICK();
#pragma omp parallel for
  for (int y = 0; y < vertex.height(); y++)
#pragma simd
    for (int x = 0; x < vertex.width(); x++) {

      Eigen::Vector2i pos(x, y);
      const Eigen::Vector3f dir =
        (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
      const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
      se::ray_iterator<T> ray(*volume._map_index, transl, dir, nearPlane, farPlane);
      ray.next();
      const float t_min = ray.tcmin(); /* Get distance to the first intersected block */
      const Eigen::Vector4f hit = t_min > 0.f ?
        raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, largestep) :
        Eigen::Vector4f::Constant(0.f);
      if(hit.w() > 0.0) {
        vertex[x + y * vertex.width()] = hit.head<3>();
        Eigen::Vector3f surfNorm = volume.grad(hit.head<3>(),
            [](const auto& val){ return val.x; });
        if (surfNorm.norm() == 0) {
          //normal[pos] = normalize(surfNorm); // APN added
          normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0, 0);
        } else {
          // Invert normals if SDF
          normal[pos.x() + pos.y() * normal.width()] = std::is_same<T, SDF>::value ?
            (-1.f * surfNorm).normalized() : surfNorm.normalized();
        }
      } else {
        vertex[pos.x() + pos.y() * vertex.width()] = Eigen::Vector3f::Constant(0);
        normal[pos.x() + pos.y() * normal.width()] = Eigen::Vector3f(INVALID, 0, 0);
      }
    }
  TOCK("raycastKernel", vertex.width() * vertex.height());
}

// void renderNormalKernel(uchar3* out, const float3* normal, uint2 normalSize) {
// 	TICK();
// 	unsigned int y;
// #pragma omp parallel for shared(out), private(y)
// 	for (y = 0; y < normalSize.y; y++)
// 		for (unsigned int x = 0; x < normalSize.x; x++) {
// 			unsigned int pos = (x + y * normalSize.x);
// 			float3 n = normal[pos];
// 			if (n.x == -2) {
// 				out[pos] = make_uchar3(0, 0, 0);
// 			} else {
// 				n = normalize(n);
// 				out[pos] = make_uchar3(n.x * 128 + 128, n.y * 128 + 128,
// 						n.z * 128 + 128);
// 			}
// 		}
// 	TOCK("renderNormalKernel", normalSize.x * normalSize.y);
// }

void renderDepthKernel(unsigned char* out, float * depth,
    const Eigen::Vector2i& depthSize, const float nearPlane,
    const float farPlane);

void renderTrackKernel(unsigned char* out,
    const TrackData* data,
    const Eigen::Vector2i& outSize);

template <typename T>
void renderVolumeKernel(const Volume<T>& volume,
    unsigned char* out, // RGBW packed
    const Eigen::Vector2i& depthSize,
    const Eigen::Matrix4f& view,
    const float nearPlane,
    const float farPlane,
    const float mu,
		const float step,
    const float largestep,
    const Eigen::Vector3f& light,
		const Eigen::Vector3f& ambient,
    bool render,
    const se::Image<Eigen::Vector3f>& vertex,
    const se::Image<Eigen::Vector3f>& normal) {
  TICK();
#pragma omp parallel for
  for (int y = 0; y < depthSize.y(); y++) {
    for (int x = 0; x < depthSize.x(); x++) {
      Eigen::Vector4f hit;
      Eigen::Vector3f test, surfNorm;
      const int idx = (x + depthSize.x()*y) * 4;

      if(render) {
        const Eigen::Vector3f dir =
          (view.topLeftCorner<3, 3>() * Eigen::Vector3f(x, y, 1.f)).normalized();
        const Eigen::Vector3f transl = view.topRightCorner<3, 1>();
        se::ray_iterator<typename Volume<T>::field_type> ray(*volume._map_index,
            transl, dir, nearPlane, farPlane);
        ray.next();
        const float t_min = ray.tmin(); /* Get distance to the first intersected block */
        hit = t_min > 0.f ?
          raycast(volume, transl, dir, t_min, ray.tmax(), mu, step, largestep) :
          Eigen::Vector4f::Constant(0.f);
        if (hit.w() > 0) {
          test = hit.head<3>();
          surfNorm = volume.grad(test, [](const auto& val){ return val.x; });

          // Invert normals if SDF
          surfNorm = std::is_same<T, SDF>::value ? -1.f * surfNorm : surfNorm;
        } else {
          surfNorm = Eigen::Vector3f(INVALID, 0, 0);
        }
      }
      else {
        test = vertex[x + depthSize.x()*y];
        surfNorm = normal[x + depthSize.x()*y];
      }

      if (surfNorm.x() != INVALID && surfNorm.norm() > 0) {
        const Eigen::Vector3f diff = (test - light).normalized();
        const Eigen::Vector3f dir = Eigen::Vector3f::Constant(fmaxf(surfNorm.normalized().dot(diff), 0.f));
        Eigen::Vector3f col = dir + ambient;
        se::math::clamp(col, Eigen::Vector3f::Constant(0.f), Eigen::Vector3f::Constant(1.f));
        col *=  255.f;
        out[idx + 0] = col.x();
        out[idx + 1] = col.y();
        out[idx + 2] = col.z();
        out[idx + 3] = 0;
      } else {
        out[idx + 0] = 0;
        out[idx + 1] = 0;
        out[idx + 2] = 0;
        out[idx + 3] = 0;
      }
    }
  }
  TOCK("renderVolumeKernel", depthSize.x() * depthSize.y());
}

#endif

