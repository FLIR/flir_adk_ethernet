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

    _imgSubscriber = _nh.subscribe<sensor_msgs::Image>("image_raw", 1,
        boost::bind(&TimeDifference::getImageHeader, this, _1));
    _actualTimeSubscriber = _nh.subscribe<std_msgs::Header>("actual_timestamp", 1,
        boost::bind(&TimeDifference::getActualTimeHeader, this, _1));
}

void TimeDifference::calculateDifferences() {
    auto sum = accumulate(_timeDifferences.begin(), _timeDifferences.end(), 0.0);
    auto avg = sum / (double)_timeDifferences.size();
    auto msAvg = avg / 1e6;

    std::cout << getName() << " - Avg timestamp difference: " << msAvg << " ms" << std::endl;
}

void TimeDifference::getImageHeader(const sensor_msgs::Image::ConstPtr& img) {
    _imgHeader = img->header;
    if(_imgHeader.seq == _actualTimeHeader.seq) {
        addTimeDiff();
    }
}

void TimeDifference::getActualTimeHeader(const std_msgs::Header::ConstPtr& msg) {
    _actualTimeHeader = *msg;
    if(_imgHeader.seq == _actualTimeHeader.seq) {
        addTimeDiff();
    }
}

void TimeDifference::addTimeDiff() {
    auto t1 = _imgHeader.stamp.nsec;
    auto t2 = _actualTimeHeader.stamp.nsec;
    auto diff = abs(t1 - t2);
    _timeDifferences.push_back((uint32_t)diff);

    if(_timeDifferences.size() >= NUM_READINGS) {
        calculateDifferences();
    }
}
