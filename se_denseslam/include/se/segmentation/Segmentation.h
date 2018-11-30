//
// Created by dorianhenning on 09/11/18.
//

#ifndef SUPEREIGHT_SEGMENTATION_H
#define SUPEREIGHT_SEGMENTATION_H

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

    SegmentationResult(int width, int height): width_(width), height_(height){
        labelImg = cv::Mat::zeros(cv::Size(height, width), CV_32SC1);
    }

    SegmentationResult(uint2 imgSize){
        width_ = imgSize.y;
        height_ = imgSize.x;
        labelImg = cv::Mat::zeros(cv::Size(imgSize.x, imgSize.y), CV_32SC1);
    }

    void reset(){
        labelImg = cv::Mat::zeros(cv::Size(height_, width_), CV_32SC1);
        instance_id_mask.clear();
        instance_class_id.clear();
    }

    int find_classID_from_instaneID(const int& instance_id, bool info) const;
    bool find_classProb_from_instID(ProbVector& all_prob, const int &instance_id,
                                    bool info) const;
    void output(const uint&frame, std::string str ) const ;
    void print_class_all_prob();
};

#endif //SUPEREIGHT_SEGMENTATION_H
