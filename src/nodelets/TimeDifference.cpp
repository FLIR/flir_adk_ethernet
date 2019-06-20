#include <pluginlib/class_list_macros.h>
#include "flir_boson_ethernet/TimeDifference.h"

PLUGINLIB_EXPORT_CLASS(flir_boson_ethernet::TimeDifference, nodelet::Nodelet)

using namespace flir_boson_ethernet;

TimeDifference::TimeDifference() {
    
}

TimeDifference::~TimeDifference() {

}

void TimeDifference::onInit() {
    _nh = getNodeHandle();
    _pnh = getPrivateNodeHandle();

    // message_filters::Subscriber<MultiTimeHeader> left(_nh, 
    //     "left/actual_timestamp", 1);
    // message_filters::Subscriber<MultiTimeHeader> right(_nh, 
    //     "right/actual_timestamp", 1);

    // message_filters::TimeSynchronizer<MultiTimeHeader, MultiTimeHeader> 
    //     sync(left, right, 1);
    // sync.registerCallback(boost::bind(&TimeDifference::calculateDifferences, 
    //     this, _1, _2));
    // _leftSubscriber = _nh.subscribe<MultiTimeHeader>("image_raw", 1,
    //     boost::bind(&TimeDifference::getImageHeader, this, _1));
    // _rightSubscriber = _nh.subscribe<MultiTimeHeader>("actual_timestamp", 1,
    //     boost::bind(&TimeDifference::getActualTimeHeader, this, _1));
}

// void TimeDifference::calculateDifferences(const MultiTimeHeaderConstPtr& leftMsg,
//     const MultiTimeHeaderConstPtr& rightMsg)
// {
//     std::cout << "here" << std::endl;
//     // auto t1 = leftMsg->actual_stamp;
//     // auto t2 = rightMsg->actual_stamp;
//     // auto diff = abs((t1 - t2).nsec);
//     // std::cout << "t1: " << t1 << ", t2: " << t2 << ", diff: " << diff << std::endl;
    
//     // auto sum = accumulate(_timeDifferences.begin(), _timeDifferences.end(), 0.0);
//     // auto avg = sum / (double)_timeDifferences.size();
//     // auto msAvg = avg / 1e6;

//     // std::cout << getName() << " - Avg timestamp difference: " << msAvg << " ms" << std::endl;
//     // _timeDifferences.clear();
// }

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
