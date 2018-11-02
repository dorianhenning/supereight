/*
 *
 * Copyright 2016 Emanuele Vespa, Imperial College London 
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
 * */
#ifndef KFUSION_MAPPING_HPP
#define KFUSION_MAPPING_HPP
#include <se/node.hpp>
//#include "opencv2/opencv.hpp"

struct sdf_update {

  template <typename DataHandlerT>
  void operator()(DataHandlerT& handler, const Eigen::Vector3i&, 
      const Eigen::Vector3f& pos, const Eigen::Vector2f& pixel) {

    const Eigen::Vector2i px = pixel.cast<int>();
    const float depthSample = depth[px(0) + depthSize(0)*px(1)];
    if (depthSample <=  0) return;
    const float diff = (depthSample - pos(2)) 
      * std::sqrt( 1 + sq(pos(0) / pos(2)) + sq(pos(1) / pos(2)));
    if (diff > -mu) {
      const float sdf = fminf(1.f, diff / mu);
      auto data = handler.get();
      // TSDF
      data.x = clamp((data.y * data.x + sdf) / (data.y + 1), -1.f, 1.f);
      data.y = fminf(data.y + 1, maxweight);

      // color
      const Eigen::Vector3f rgb_measured = rgb[px(0) + depthSize(0)*px(1)];
      data.r = (rgb_measured(0) + data.r * data.cw)/(data.cw + 1);
      data.g = (rgb_measured(0) + data.g * data.cw)/(data.cw + 1);
      data.b = (rgb_measured(0) + data.b * data.cw)/(data.cw + 1);
      data.cw ++;

      // foreground probability
//      const float fg_measured = mast.at<float>(px(1), px(0));   // Eigen to OpenCV conversion
//      data.fg = (fg_measured + data.fg  data.w)/(data.v + 1);
//      data.v ++;

      handler.set(data);
    }
  } 

  sdf_update(const float * d, const Eigen::Vector2i framesize, float m, int mw) : 
    depth(d), depthSize(framesize), mu(m), maxweight(mw){};

  const float * depth;
  const Eigen::Vector3f * rgb;
//  const cv::Mat mask;
  Eigen::Vector2i depthSize;
  float mu;
  int maxweight;
};

#endif
