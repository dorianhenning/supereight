//
// Created by dorianhenning on 09/11/18.
//

#ifndef SUPEREIGHT_SEGMENTATION_H
#define SUPEREIGHT_SEGMENTATION_H

#include <map>
#include <vector>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

struct SegmentationResult {
    std::map<int, cv::Mat> instanceIDmask_;
    cv::Mat labelImg_;
    std::map<int, int> instanceClassID_;
    std::map<int, ProbVector, std::less<int>,
            Eigen::aligned_allocator<std::pair<const int, ProbVector> > >
            instanceIDallProb_;
    unsigned int labelNumber_;
    void combineLabels();
    void generateBGlabels();

    int width_;
    int height_;

    SegmentationResult(int width, int height): width_(width), height_(height)
    {
        labelImg_ = cv::Mat::zeros(cv::Size(height, width), CV_32SC1);
    }

    SegmentationResult(Eigen::Vector2i& imgSize)
    {
        width_ = imgSize.y();
        height_ = imgSize.x();
        labelImg_ = cv::Mat::zeros(cv::Size(height_, width_), CV_32SC1);
    }

    void reset()
    {
        labelImg_ = cv::Mat::zeros(cv::Size(height_, width_), CV_32SC1);
        instanceIDmask_.clear();
        instanceClassID_.clear();
    }

    int findClassIDfromInstanceID(const int& instance_id, bool info) const;
    bool findClassProbFromInstanceID(ProbVector& all_prob, const int &instance_id,
                                    bool info) const;
    void output(const uint&frame, std::string str ) const;
    void printClassProbVector();
};

class Segmentation {
public:


private:
};

#endif //SUPEREIGHT_SEGMENTATION_H
