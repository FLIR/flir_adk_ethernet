/******************************************************************************/
/*                                                                            */
/*  Copyright (C) 2018, FLIR Systems                                          */
/*  All rights reserved.                                                      */
/*                                                                            */
/******************************************************************************/
#include <pluginlib/class_list_macros.h>
#include "flir_adk_ethernet/TimeDifference.h"

PLUGINLIB_EXPORT_CLASS(flir_adk_ethernet::TimeDifference, nodelet::Nodelet)

using namespace flir_adk_ethernet;

TimeDifference::TimeDifference() {
    
}

TimeDifference::~TimeDifference() {

}

void TimeDifference::onInit() {
    _nh = getNodeHandle();
    _pnh = getPrivateNodeHandle();

    _leftSub = _nh.subscribe<MultiTimeHeader>("left/actual_timestamp", 1, 
        boost::bind(&TimeDifference::calculateDifferences, this, _1,
            &_leftHeader, &_rightHeader));
    _rightSub = _nh.subscribe<MultiTimeHeader>("right/actual_timestamp", 1, 
        boost::bind(&TimeDifference::calculateDifferences, this, _1,
            &_rightHeader, &_leftHeader));
    // message_filters::Subscriber<MultiTimeHeader> right(_nh, 
    //     "right/actual_timestamp", 1);


    // message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(1), 
    //     left, right);

    // _conn = sync.registerCallback(boost::bind(
    //     &TimeDifference::calculateDifferences, this, _1, _2));
}

void TimeDifference::calculateDifferences(const MultiTimeHeaderConstPtr& msg,
    MultiTimeHeader *header, MultiTimeHeader *otherHeader)
{
    *header = *msg;
    if(header->header.stamp == otherHeader->header.stamp) {
        auto t1 = header->actual_stamp;
        auto t2 = otherHeader->actual_stamp;
        auto diff = abs((t1 - t2).nsec);
        std::cout << "t1: " << t1 << ", t2: " << t2 << ", diff: " << diff << std::endl;
    }
    
    // auto sum = accumulate(_timeDifferences.begin(), _timeDifferences.end(), 0.0);
    // auto avg = sum / (double)_timeDifferences.size();
    // auto msAvg = avg / 1e6;

    // std::cout << getName() << " - Avg timestamp difference: " << msAvg << " ms" << std::endl;
    // _timeDifferences.clear();
}

// void TimeDifference::getImageHeader(const sensor_msgs::Image::ConstPtr& img) {
//     _imgHeader = img->header;
//     if(_imgHeader.seq == _actualTimeHeader.seq) {
//         addTimeDiff();
//     }
// }

// void TimeDifference::getActualTimeHeader(const std_msgs::Header::ConstPtr& msg) {
//     _actualTimeHeader = *msg;
//     if(_imgHeader.seq == _actualTimeHeader.seq) {
//         addTimeDiff();
//     }
// }

// void TimeDifference::addTimeDiff() {
//     int64_t t1 = _imgHeader.stamp.nsec;
//     int64_t t2 = _actualTimeHeader.stamp.nsec;

//     // temporary correction for Spinnaker bug giving 0 time
//     if(t2 == 0) {
//         return;
//     }

//     double diff = abs(t1 - t2);
//     // std::cout << "t1: " << t1 << ", t2: " << t2 << ", diff: " << diff << std::endl;
//     _timeDifferences.push_back((uint32_t)diff);

//     if(_timeDifferences.size() >= NUM_READINGS) {
//         calculateDifferences();
//     }
// }
