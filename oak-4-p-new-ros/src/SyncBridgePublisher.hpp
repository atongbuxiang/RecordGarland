#pragma once

#include <algorithm>
#include <deque>
#include <chrono>
#include <cstdint>
#include <thread>
#include <type_traits>
#include <typeinfo>
#include <vector>
#include <map>

#include <time.h>

#include <future>
#include <atomic>
#include <functional>

#include "boost/make_shared.hpp"
#include "boost/shared_ptr.hpp"
#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/depthai.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace dai {

namespace ros {

namespace StdMsgs = std_msgs::msg;
namespace ImageMsgs = sensor_msgs::msg;
using ImagePtr = ImageMsgs::Image::SharedPtr;
namespace rosOrigin = ::rclcpp;

std::string formatDuration(std::chrono::steady_clock::duration duration) {
    // 首先，将 duration 转换为 nanoseconds
    auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

    // 计算小时、分钟、秒和毫秒
    auto const hrs = std::chrono::duration_cast<std::chrono::hours>(ns);
    ns -= hrs;
    auto mins = std::chrono::duration_cast<std::chrono::minutes>(ns);
    ns -= mins;
    auto secs = std::chrono::duration_cast<std::chrono::seconds>(ns);
    ns -= secs;
    auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(ns);
    ns -= millis;
    auto micros = std::chrono::duration_cast<std::chrono::microseconds>(ns);
    ns -= micros;

    // 构建格式化的字符串
    std::ostringstream oss;
    oss << std::setw(2) << std::setfill('0') << hrs.count() << ":"   // 时
        << std::setw(2) << std::setfill('0') << mins.count() << ":"  // 分
        << std::setw(2) << std::setfill('0') << secs.count() << "."  // 秒
        << std::setw(3) << std::setfill('0') << millis.count()       // 毫秒
        << std::setw(3) << std::setfill('0') << micros.count();

    return oss.str();
}

template <class RosMsg>
class SyncBridgePublisher {
   public:
    using ConvertFunc = std::function<ImageMsgs::Image(std::shared_ptr<dai::ImgFrame>, const sensor_msgs::msg::CameraInfo&)>;
    using MessageGroupCallback = std::function<void(const std::shared_ptr<dai::MessageGroup>&)>;

    using CustomPublisher = typename rclcpp::Publisher<RosMsg>::SharedPtr;

    SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::map<std::string, std::string> rosTopic,
                    ConvertFunc converter,
                    int queueSize,
                    std::string cameraParamUri = "",
                    bool lazyPublisher = true);

    SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                    std::shared_ptr<rclcpp::Node> node,
                    std::map<std::string, std::string> rosTopics,
                    ConvertFunc converter,
                    int queueSize,
                    ImageMsgs::CameraInfo cameraInfoData,
                    bool lazyPublisher = true);

    /**
     * Tag Dispacher function to to overload the Publisher to ImageTransport Publisher
     */
    void advertise(int queueSize, std::true_type);

    /**
     * Tag Dispacher function to to overload the Publisher to use Default ros::Publisher
     */
    void advertise(int queueSize, std::false_type);

    SyncBridgePublisher(const SyncBridgePublisher& other);

    void addPublisherCallback();

    void publishHelper(std::shared_ptr<dai::MessageGroup> inData);

    void image_converter(std::string camera_name, std::shared_ptr<dai::ImgFrame> inData, std::map<std::string, RosMsg> &opMsgs);

    void startPublisherThread();

    void setMessageGroupCallback(MessageGroupCallback callback);

    ~SyncBridgePublisher();

   private:
    /**
     * adding this callback will allow you to still be able to consume
     * the data for other processing using get() function .
     */
    void daiCallback(std::string name, std::shared_ptr<ADatatype> data);
    void recordFrameAndPrintFps(const std::string& camera_name);

    static const std::string LOG_TAG;
    std::shared_ptr<dai::DataOutputQueue> _daiMessageQueue;
    ConvertFunc _converter;

    std::shared_ptr<rclcpp::Node> _node;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr> _cameraInfoPublisher;

    image_transport::ImageTransport _it;
    ImageMsgs::CameraInfo _cameraInfoData;
    std::map<std::string, CustomPublisher> _rosPublisher;

    std::thread _readingThread;
    std::map<std::string, std::string> _rosTopics;
    std::string _camInfoFrameId, _cameraParamUri;
    std::map<std::string, std::unique_ptr<camera_info_manager::CameraInfoManager>> _camInfoManager;
    bool _isCallbackAdded = false;
    bool _isImageMessage = false;  // used to enable camera info manager
    bool _lazyPublisher = true;
    std::atomic<bool> _running{false};
    MessageGroupCallback _messageGroupCallback;
    // uint32_t fps = 0;
    // time_t start_ = time(NULL);
    std::mutex opMsgs_mutex;
    std::mutex fps_mutex;
    std::map<std::string, uint64_t> _fpsFrameCounts;
    std::chrono::steady_clock::time_point _fpsLastPrint = std::chrono::steady_clock::now();
};

template <class RosMsg>
const std::string SyncBridgePublisher<RosMsg>::LOG_TAG = "SyncBridgePublisher";

template <class RosMsg>
SyncBridgePublisher<RosMsg>::SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::map<std::string, std::string> rosTopics,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 std::string cameraParamUri,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _converter(converter),
      _node(node),
      _it(_node),
      _rosTopics(rosTopics),
      _cameraParamUri(cameraParamUri),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg>
SyncBridgePublisher<RosMsg>::SyncBridgePublisher(std::shared_ptr<dai::DataOutputQueue> daiMessageQueue,
                                                 std::shared_ptr<rclcpp::Node> node,
                                                 std::map<std::string, std::string> rosTopics,
                                                 ConvertFunc converter,
                                                 int queueSize,
                                                 ImageMsgs::CameraInfo cameraInfoData,
                                                 bool lazyPublisher)
    : _daiMessageQueue(daiMessageQueue),
      _node(node),
      _converter(converter),
      _it(_node),
      _cameraInfoData(cameraInfoData),
      _rosTopics(rosTopics),
      _lazyPublisher(lazyPublisher) {
    // ROS_DEBUG_STREAM_NAMED(LOG_TAG, "Publisher Type : " << typeid(CustomPublisher).name());
    advertise(queueSize, std::is_same<RosMsg, ImageMsgs::Image>{});
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::advertise(int queueSize, std::false_type) {
    for (auto& rosTopic: _rosTopics) {
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        _rosPublisher[rosTopic.first] = _node->create_publisher<RosMsg>(rosTopic.second, rclcpp::QoS(queueSize), options);
        // _rosPublisher[rosTopic.first] = std::make_shared<rosOrigin::Publisher>(_nh.advertise<RosMsg>(rosTopic.second, queueSize));
    }
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::advertise(int queueSize, std::true_type) {
    const size_t imageQueueDepth = static_cast<size_t>(std::max(queueSize, 3));
    rmw_qos_profile_t qos_sensor_data = {
        .history = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
        .depth = imageQueueDepth,
        .reliability = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
        .durability = RMW_QOS_POLICY_DURABILITY_VOLATILE,
        .deadline = {0, 0},
        .lifespan = {0, 0},
        .liveliness = RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
        .liveliness_lease_duration = {0, 0},
        .avoid_ros_namespace_conventions = false
    };
    for (auto& rosTopic: _rosTopics) {
        auto cam_name = rosTopic.first;
        auto topic = rosTopic.second;
        if(!cam_name.empty()) {
            _isImageMessage = true;
            _camInfoManager[cam_name] = std::make_unique<camera_info_manager::CameraInfoManager>(_node.get(), cam_name, _cameraParamUri);
            if(_cameraParamUri.empty()) {
                _camInfoManager[cam_name]->setCameraInfo(_cameraInfoData);
            }
            // _cameraInfoPublisher[cam_name] = std::make_shared<rosOrigin::Publisher>(_nh.advertise<ImageMsgs::CameraInfo>(cam_name + "/camera_info", queueSize));
            rclcpp::PublisherOptions options;
            options.qos_overriding_options = rclcpp::QosOverridingOptions();
            _cameraInfoPublisher.insert({cam_name, _node->create_publisher<ImageMsgs::CameraInfo>(cam_name + "/camera_info", queueSize, options)});
        }
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        _rosPublisher[cam_name] = _node->create_publisher<RosMsg>(topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_sensor_data), qos_sensor_data), options);
        // _rosPublisher[cam_name] = std::make_shared<image_transport::Publisher>(_it.advertise(topic, queueSize));
    }
}

// template <class RosMsg>
// SyncBridgePublisher<RosMsg>::SyncBridgePublisher(const SyncBridgePublisher& other) {
//     _daiMessageQueue = other._daiMessageQueue;
//     _node = other._node;
//     _converter = other._converter;
//     _rosTopics = other._rosTopics;
//     _it = other._it;
//     for (auto& rosTopic: _rosTopics) {
//         auto cam_name = rosTopic.first;
//         _rosPublisher[cam_name] = CustomPublisher(other._rosPublisher.at(cam_name));

//         if(other._isImageMessage) {
//             _isImageMessage = true;
//             _camInfoManager[cam_name] = std::make_unique<camera_info_manager::CameraInfoManager>(std::move(other._camInfoManager.at(cam_name)));
//             _cameraInfoPublisher[cam_name] = rosOrigin::Publisher(other._cameraInfoPublisher.at(cam_name));
//         }
//     }
// }

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::daiCallback(std::string name, std::shared_ptr<ADatatype> data) {
    auto daiDataPtr = std::dynamic_pointer_cast<dai::MessageGroup>(data);
    for (auto& rosTopic: _rosTopics) {
        auto pkg = daiDataPtr->get<dai::ImgFrame>(rosTopic.first);
        publishHelper(pkg, rosTopic.first);
    }
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::startPublisherThread() {
    if(_isCallbackAdded) {
        std::runtime_error(
            "addPublisherCallback() function adds a callback to the"
            "depthai which handles the publishing so no need to start"
            "the thread using startPublisherThread() ");
    }

    _running.store(true);
    auto context = _node->get_node_base_interface()->get_context();
    _readingThread = std::thread([&, context]() {
        int messageCounter = 0;
        while(_running.load() && rosOrigin::ok(context)) {
            // auto daiDataPtr = _daiMessageQueue->get<dai::MessageGroup>();
            auto daiDataPtr = _daiMessageQueue->tryGet<dai::MessageGroup>();
            if(daiDataPtr == nullptr) {
                messageCounter++;
                if(messageCounter > 2000000) {
                    messageCounter = 0;
                }
                continue;
            }

            if(messageCounter != 0) {
                messageCounter = 0;
            }
            publishHelper(daiDataPtr);
            // fps++;
            // if (time(NULL) - start_ >= 1) {
            //     printf("fps: %d\n", fps);
            //     fps = 0;
            //     start_ = time(NULL);
            // }
        }
    });
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::addPublisherCallback() {
    _daiMessageQueue->addCallback(std::bind(&SyncBridgePublisher<RosMsg>::daiCallback, this, std::placeholders::_1, std::placeholders::_2));
    _isCallbackAdded = true;
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::image_converter(std::string camera_name, std::shared_ptr<dai::ImgFrame> inData, std::map<std::string, RosMsg> &opMsgs) {
    auto outImageMsg = _converter(inData, sensor_msgs::msg::CameraInfo());
    std::lock_guard<std::mutex> lock(opMsgs_mutex);
    opMsgs[camera_name] = std::move(outImageMsg);
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::setMessageGroupCallback(MessageGroupCallback callback) {
    _messageGroupCallback = std::move(callback);
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::recordFrameAndPrintFps(const std::string& camera_name) {
    std::lock_guard<std::mutex> lock(fps_mutex);

    _fpsFrameCounts[camera_name]++;
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration<double>(now - _fpsLastPrint).count();
    if (elapsed < 1.0) {
        return;
    }

    std::ostringstream fpsStream;
    fpsStream << std::fixed << std::setprecision(2) << "[camera fps]";
    for (const auto& rosTopic : _rosTopics) {
        const auto& name = rosTopic.first;
        auto countIt = _fpsFrameCounts.find(name);
        double fps = countIt == _fpsFrameCounts.end() ? 0.0 : static_cast<double>(countIt->second) / elapsed;
        fpsStream << " " << name << "=" << fps;
        _fpsFrameCounts[name] = 0;
    }
    std::cout << fpsStream.str() << std::endl;
    _fpsLastPrint = now;
}

template <class RosMsg>
void SyncBridgePublisher<RosMsg>::publishHelper(std::shared_ptr<dai::MessageGroup> inDataPtr) {
    // std::deque<RosMsg> opMsgs;
    std::map<std::string, int> infoSubCount_map, mainSubCount_map;
    std::map<std::string, RosMsg> opMsgs;
    std::vector<std::future<void>> results;

    if (_messageGroupCallback) {
        _messageGroupCallback(inDataPtr);
    }

    for (auto& rosTopic: _rosTopics) {
        std::string camera_name = rosTopic.first;
        int infoSubCount = 0, mainSubCount = 0;
        if(_isImageMessage) {
            // infoSubCount = _cameraInfoPublisher.at(camera_name)->getNumSubscribers();
            infoSubCount = _node->count_subscribers(camera_name + "/camera_info");
        }
        // mainSubCount = _rosPublisher.at(camera_name)->getNumSubscribers();
        mainSubCount = _node->count_subscribers(rosTopic.second);

        if(!_lazyPublisher || (mainSubCount > 0 || infoSubCount > 0)) {
            auto pkg = inDataPtr->get<dai::ImgFrame>(camera_name);
            recordFrameAndPrintFps(camera_name);
            // _converter(pkg, opMsgs);
            results.emplace_back(std::async(std::launch::async, 
                &dai::ros::SyncBridgePublisher<RosMsg>::image_converter, 
                this, camera_name, pkg, std::ref(opMsgs)));
        }
        infoSubCount_map.insert({camera_name, infoSubCount});
        mainSubCount_map.insert({camera_name, mainSubCount});
    }

    for (auto& future : results) {
        future.get();
    }

    for (auto& rosTopic: _rosTopics) {
        if (!opMsgs.size())
            break;
        std::string camera_name = rosTopic.first;
        const auto stamp = opMsgs[camera_name].header.stamp;
        const auto frame_id = opMsgs[camera_name].header.frame_id;
        if(mainSubCount_map[camera_name] > 0) {
            _rosPublisher.at(camera_name)->publish(std::make_unique<RosMsg>(std::move(opMsgs[camera_name])));
        }

        if(infoSubCount_map[camera_name] > 0) {
            // if (_isImageMessage){
            //     _camInfoFrameId = curr.header.frame_id
            // }
            auto localCameraInfo = _camInfoManager.at(camera_name)->getCameraInfo();
            // localCameraInfo.header.seq = currMsg.header.seq;
            localCameraInfo.header.stamp = stamp;
            localCameraInfo.header.frame_id = frame_id;
            _cameraInfoPublisher.at(camera_name)->publish(localCameraInfo);
        }
        // opMsgs.pop_front();
        opMsgs.erase(camera_name);
    }
}

template <class RosMsg>
SyncBridgePublisher<RosMsg>::~SyncBridgePublisher() {
    _running.store(false);
    if(_readingThread.joinable()) _readingThread.join();
}

}  // namespace ros

namespace rosBridge = ros;

}  // namespace dai
