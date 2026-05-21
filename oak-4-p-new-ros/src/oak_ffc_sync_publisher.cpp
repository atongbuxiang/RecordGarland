#include <map>
#include <vector>
#include <tuple>
#include <memory>

#include "camera_info_manager/camera_info_manager.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "stereo_msgs/msg/disparity_image.hpp"
#include "sensor_msgs/msg/imu.hpp"

// Inludes common necessary includes for development using depthai library
#include "depthai/device/DataQueue.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/ColorCamera.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/BridgePublisher.hpp"
#include "depthai_bridge/DisparityConverter.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/ImuConverter.hpp"
#include "SyncBridgePublisher.hpp"
#include "rclcpp_components/register_node_macro.hpp"

struct oak_ffc_ros_opts_t
{
    std::string tfPrefix;
    std::string camera_type;
    std::string cameraParamUri;
    std::string resolution;
    std::string camera_name;
    int fps;
    int compressed;
    std::string oakFwUri;
    int imu_hz;
    std::vector<std::string> cam_board_sockets;
    int sync_threshold;
    int isp_scale;
};

std::map<std::string, std::tuple<dai::node::MonoCamera::Properties::SensorResolution, int, int>> mono_res_opts = {
    {"400p", {dai::node::MonoCamera::Properties::SensorResolution::THE_400_P, 640, 400}},
    {"480p", {dai::node::MonoCamera::Properties::SensorResolution::THE_480_P, 640, 480}},
    {"720p", {dai::node::MonoCamera::Properties::SensorResolution::THE_720_P, 1280, 720}},
    {"800p", {dai::node::MonoCamera::Properties::SensorResolution::THE_800_P, 1280, 800}},
    {"1200p", {dai::node::MonoCamera::Properties::SensorResolution::THE_1200_P, 1920, 1200}}
};

std::map<std::string, std::tuple<dai::node::ColorCamera::Properties::SensorResolution, int, int>> color_res_opts = {
    {"480p", {dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P, 640, 480}},
    {"720p", {dai::node::ColorCamera::Properties::SensorResolution::THE_720_P, 1280, 720}},
    {"800p", {dai::node::ColorCamera::Properties::SensorResolution::THE_800_P, 1280, 800}},
    {"1080p", {dai::node::ColorCamera::Properties::SensorResolution::THE_1080_P, 1920, 1080}},
    {"1200p", {dai::node::ColorCamera::Properties::SensorResolution::THE_1200_P, 1920, 1200}},
    {"4k", {dai::node::ColorCamera::Properties::SensorResolution::THE_4_K, 3840, 2160}}
    // {"5mp": dai::node::ColorCamera::Properties::SensorResolution::THE_5_MP},
    // {"12mp": dai::node::ColorCamera::Properties::SensorResolution::THE_12_MP},
    // {"48mp": dai::node::ColorCamera::Properties::SensorResolution::THE_48_MP}
};

std::map<std::string, std::tuple<dai::CameraBoardSocket, std::string>> cam_socket_opts = {
    {"CAM_A", {dai::CameraBoardSocket::CAM_A, "CAM_A/image"}},
    {"CAM_B", {dai::CameraBoardSocket::CAM_B, "CAM_B/image"}},
    {"CAM_C", {dai::CameraBoardSocket::CAM_C, "CAM_C/image"}},
    {"CAM_D", {dai::CameraBoardSocket::CAM_D, "CAM_D/image"}},
};

std::tuple<dai::Pipeline, int, int >createPipeline(oak_ffc_ros_opts_t &ros_opts) {
    dai::Pipeline pipeline;
    pipeline.setXLinkChunkSize(0);

    int width, height;
    auto sync = pipeline.create<dai::node::Sync>();
    sync->setSyncThreshold(std::chrono::milliseconds(ros_opts.sync_threshold));
    auto xSyncOut = pipeline.create<dai::node::XLinkOut>();
    xSyncOut->setStreamName("sync");
    sync->out.link(xSyncOut->input);

    for (const auto& board_socket_name: ros_opts.cam_board_sockets) {
        // auto xout = pipeline.create<dai::node::XLinkOut>();
        // xout->setStreamName(opts.first);
        auto opts = cam_socket_opts.find(board_socket_name);
        if (opts == cam_socket_opts.end())
            continue;
        auto board_socket = std::get<0>(opts->second);
        auto videoEncs = pipeline.create<dai::node::VideoEncoder>();
        if (ros_opts.compressed) {
            videoEncs->setDefaultProfilePreset(ros_opts.fps, dai::VideoEncoderProperties::Profile::MJPEG);
            videoEncs->bitstream.link(sync->inputs[board_socket_name]);
        }
        if (ros_opts.camera_type == "color") {
            dai::node::ColorCamera::Properties::SensorResolution rgbResolution;
            auto camera = pipeline.create<dai::node::ColorCamera>();
            auto it = color_res_opts.find(ros_opts.resolution);
            if (it != color_res_opts.end()) {
                std::tie(rgbResolution, width, height) = it->second;
                camera->setResolution(rgbResolution);
                if (ros_opts.isp_scale > 1) {
                    camera->setIspScale(1, ros_opts.isp_scale);
                    width /= ros_opts.isp_scale;
                    height /= ros_opts.isp_scale;
                }
            }
            // camera->isp.link(xout->input);
            camera->setBoardSocket(board_socket);
            camera->setFps(ros_opts.fps);
            if (ros_opts.compressed)
                camera->video.link(videoEncs->input);
            else
                camera->isp.link(sync->inputs[board_socket_name]);
            if (ros_opts.camera_name == "ov9782") {
                if (board_socket_name == ros_opts.cam_board_sockets[0]) {
                    camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::OUTPUT);
                } else
                    camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
            } else 
                camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
            
        } else {
            dai::node::MonoCamera::Properties::SensorResolution monoResolution;
            auto camera = pipeline.create<dai::node::MonoCamera>();
            auto it = mono_res_opts.find(ros_opts.resolution);
            if (it != mono_res_opts.end()) {
                std::tie(monoResolution, width, height) = it->second;
                camera->setResolution(monoResolution);
            }
            if (ros_opts.compressed)
                camera->out.link(videoEncs->input);
            else
                camera->out.link(sync->inputs[board_socket_name]);
            camera->setBoardSocket(board_socket);
            camera->setFps(ros_opts.fps);
            if (ros_opts.camera_name == "ov9782") {
                if (board_socket_name == ros_opts.cam_board_sockets[0]) {
                    camera->initialControl.setFrameSyncMode(
                        dai::CameraControl::FrameSyncMode::OUTPUT
                    );
                } else
                    camera->initialControl.setFrameSyncMode(
                        dai::CameraControl::FrameSyncMode::INPUT
                    );
            } else
                camera->initialControl.setFrameSyncMode(dai::CameraControl::FrameSyncMode::INPUT);
        }
    }
    auto imu = pipeline.create<dai::node::IMU>();
    auto xImuOut = pipeline.create<dai::node::XLinkOut>();
    xImuOut->setStreamName("imu");
    // imu->enableIMUSensor(dai::IMUSensor::ROTATION_VECTOR, 100);
    // imu->enableIMUSensor(dai::IMUSensor::ACCELEROMETER_RAW, 100);
    // imu->enableIMUSensor(dai::IMUSensor::GYROSCOPE_RAW, 100);
    imu->enableIMUSensor({
        dai::IMUSensor::ROTATION_VECTOR,
        dai::IMUSensor::ACCELEROMETER_RAW,
        dai::IMUSensor::GYROSCOPE_RAW
    }, ros_opts.imu_hz);
    imu->setBatchReportThreshold(1);
    imu->setMaxBatchReports(10);
    imu->out.link(xImuOut->input);

    if (ros_opts.camera_name == "ov9782") {
        auto boardConfig = dai::BoardConfig();
        boardConfig.gpio[42] =
            dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::INPUT, dai::BoardConfig::GPIO::HIGH, dai::BoardConfig::GPIO::PULL_DOWN);
        pipeline.setBoardConfig(boardConfig);
    } else {
        auto script = pipeline.create<dai::node::Script>();
        script->setProcessor(dai::ProcessorType::LEON_CSS);
        script->setScript(std::string(R"(# coding=utf-8
import time
import GPIO

# Script static arguments
fps = )") + std::to_string(ros_opts.fps) + std::string(R"(
calib = Device.readCalibration2().getEepromData()
prodName  = calib.productName
boardName = calib.boardName
boardRev  = calib.boardRev

node.warn(f'Product name  : {prodName}')
node.warn(f'Board name    : {boardName}')
node.warn(f'Board revision: {boardRev}')

revision = -1
# Very basic parsing here, TODO improve
if len(boardRev) >= 2 and boardRev[0] == 'R':
    revision = int(boardRev[1])
node.warn(f'Parsed revision number: {revision}')

# Defaults for OAK-FFC-4P older revisions (<= R5)
GPIO_FSIN_2LANE = 41  # COM_AUX_IO2
GPIO_FSIN_4LANE = 40
GPIO_FSIN_MODE_SELECT = 6  # Drive 1 to tie together FSIN_2LANE and FSIN_4LANE

if revision >= 6:
    GPIO_FSIN_2LANE = 41  # still COM_AUX_IO2, no PWM capable
    GPIO_FSIN_4LANE = 42  # also not PWM capable
    GPIO_FSIN_MODE_SELECT = 38  # Drive 1 to tie together FSIN_2LANE and FSIN_4LANE
# Note: on R7 GPIO_FSIN_MODE_SELECT is pulled up, driving high isn't necessary (but fine to do)

# GPIO initialization
GPIO.setup(GPIO_FSIN_2LANE, GPIO.OUT)
GPIO.write(GPIO_FSIN_2LANE, 0)

GPIO.setup(GPIO_FSIN_4LANE, GPIO.IN)

GPIO.setup(GPIO_FSIN_MODE_SELECT, GPIO.OUT)
GPIO.write(GPIO_FSIN_MODE_SELECT, 1)

period = 1 / fps
active = 0.001

node.warn(f'FPS: {fps}  Period: {period}')

withInterrupts = False
if withInterrupts:
    node.critical(f'[TODO] FSYNC with timer interrupts (more precise) not implemented')
else:
    overhead = 0.003  # Empirical, TODO add thread priority option!
    while True:
        GPIO.write(GPIO_FSIN_2LANE, 1)
        time.sleep(active)
        GPIO.write(GPIO_FSIN_2LANE, 0)
        time.sleep(period - active - overhead)
)"));
    }
    return std::make_tuple(pipeline, width, height);
}

class OakFfcSyncNode {
public:
    explicit OakFfcSyncNode(const rclcpp::NodeOptions& options)
        : node_(std::make_shared<rclcpp::Node>("oak_ffc_sync", options)) {
        start();
    }

    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
        return node_->get_node_base_interface();
    }

private:
    void start() {

    dai::Pipeline pipeline;
    int width, height;

    oak_ffc_ros_opts_t ros_opts = {
        .tfPrefix = "",
        .camera_type = "color",
        .cameraParamUri = "",
        .resolution = "1200p",
        .camera_name = "ar0234",
        .fps = 30,
        .compressed = 1,
        .oakFwUri = "",
        .imu_hz = 200,
        .cam_board_sockets = {"CAM_A", "CAM_B", "CAM_C", "CAM_D"},
        .sync_threshold = 50,
        .isp_scale = 3
    };

    node_->declare_parameter("camera_name", ros_opts.camera_name);
    node_->declare_parameter("camera_param_uri", ros_opts.cameraParamUri);
    node_->declare_parameter("tf_prefix", ros_opts.tfPrefix);
    node_->declare_parameter("camera_type", ros_opts.camera_type);
    node_->declare_parameter("resolution", ros_opts.resolution);
    node_->declare_parameter("fps", ros_opts.fps);
    node_->declare_parameter("oak_fw_uri", ros_opts.oakFwUri);
    node_->declare_parameter("compressed", ros_opts.compressed);
    node_->declare_parameter("imu_hz", ros_opts.imu_hz);
    node_->declare_parameter("cam_board_sockets", ros_opts.cam_board_sockets);
    node_->declare_parameter("sync_threshold", ros_opts.sync_threshold);
    node_->declare_parameter("isp_scale", ros_opts.isp_scale);

    node_->get_parameter("camera_name", ros_opts.camera_name);
    ros_opts.cameraParamUri = node_->get_parameter("camera_param_uri").as_string();
    ros_opts.tfPrefix = node_->get_parameter("tf_prefix").as_string();
    ros_opts.camera_type = node_->get_parameter("camera_type").as_string();
    ros_opts.resolution = node_->get_parameter("resolution").as_string();
    ros_opts.fps = node_->get_parameter("fps").as_int();
    ros_opts.oakFwUri = node_->get_parameter("oak_fw_uri").as_string();
    ros_opts.compressed = node_->get_parameter("compressed").as_int();
    ros_opts.imu_hz = node_->get_parameter("imu_hz").as_int();
    ros_opts.cam_board_sockets = node_->get_parameter("cam_board_sockets").as_string_array();
    ros_opts.sync_threshold = node_->get_parameter("sync_threshold").as_int();
    ros_opts.isp_scale = node_->get_parameter("isp_scale").as_int();

    if (ros_opts.isp_scale < 1) {
        std::cerr << "\033[31m" << "isp_scale must be >= 1." << "\033[0m" << '\n';
        throw std::runtime_error("isp_scale must be >= 1.");
    }

    if (ros_opts.cam_board_sockets.size() < 1) {
        std::cerr << "\033[31m" << "Please set up at least one board camera socket." << "\033[0m" << '\n';
        throw std::runtime_error("Please set up at least one board camera socket.");
    }

    if (setenv("DEPTHAI_DEVICE_BINARY", ros_opts.oakFwUri.c_str(), 1) == 0) {
        std::cout << "DEPTHAI_DEVICE_BINARY=" << ros_opts.oakFwUri << std::endl;
    } else {
        std::cerr << "\033[31m" << "DEPTHAI_DEVICE_BINARY Environment variable setting failed!" << "\033[0m" << '\n';
    }

    for (std::string socket : ros_opts.cam_board_sockets) {
        printf("cam board socket: %s\n", socket.c_str());
    }

    std::tie(pipeline, width, height) = createPipeline(ros_opts);

    printf("width: %d height: %d\n", width, height);

    device_ = std::make_unique<dai::Device>(pipeline);

    auto calibrationHandler = device_->readCalibration();

    auto boardName = calibrationHandler.getEepromData().boardName;
    if(height > 480 && boardName == "OAK-D-LITE") {
        width = 640;
        height = 480;
    }

    std::string color_url = ros_opts.cameraParamUri + "/" + "ffc_cam.yaml";

    auto imuQueue = device_->getOutputQueue("imu", ros_opts.fps, false);
    imu_converter_ = std::make_unique<dai::rosBridge::ImuConverter>(ros_opts.tfPrefix + "_imu_frame", dai::ros::ImuSyncMethod::COPY, 0, 0, 0, 0, true);
    imu_publish_ = std::make_unique<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>>(
        imuQueue,
        node_,
        std::string("imu"),
        std::bind(&dai::rosBridge::ImuConverter::toRosMsg, imu_converter_.get(), std::placeholders::_1, std::placeholders::_2),
        ros_opts.fps
    );
    imu_publish_->addPublisherCallback();

    std::map<std::string, std::string> rosTocips;
    for (auto &board_socket_name: ros_opts.cam_board_sockets) {
        auto opts = cam_socket_opts.find(board_socket_name);
        if (opts == cam_socket_opts.end())
            continue;
        auto tocip = std::get<1>(opts->second);
        rosTocips.insert({board_socket_name, tocip});
    }

    auto syncQueue = device_->getOutputQueue("sync", 1, false);
    converter_ = std::make_unique<dai::rosBridge::ImageConverter>(ros_opts.tfPrefix + "_rgb_camera_optical_frame", true);
    if (ros_opts.compressed)
        converter_->convertFromBitstream(dai::RawImgFrame::Type::BGR888i);
    try {
        auto cameraInfo = converter_->calibrationToCameraInfo(calibrationHandler, std::get<0>(cam_socket_opts[ros_opts.cam_board_sockets[0]]), width, height);
        cam_publish_ = std::make_unique<dai::rosBridge::SyncBridgePublisher<sensor_msgs::msg::Image>>(
                        syncQueue,
                        node_,
                        rosTocips,
                        std::bind(&dai::rosBridge::ImageConverter::toRosMsgRawPtr, converter_.get(), std::placeholders::_1, std::placeholders::_2),
                        ros_opts.fps,
                        cameraInfo);
        // camPublish.addPublisherCallback();
        cam_publish_->startPublisherThread();
    } catch(const std::exception& e) {
        std::cerr << "\033[31m" << "[warning] The camera is not calibrated, please calibrate it before use!" << "\033[0m" << '\n';
        cam_publish_ = std::make_unique<dai::rosBridge::SyncBridgePublisher<sensor_msgs::msg::Image>>(
                        syncQueue,
                        node_,
                        rosTocips,
                        std::bind(&dai::rosBridge::ImageConverter::toRosMsgRawPtr, converter_.get(), std::placeholders::_1, std::placeholders::_2),
                        ros_opts.fps,
                        color_url);
        // camPublish.addPublisherCallback();
        cam_publish_->startPublisherThread();
    }
    }

    std::shared_ptr<rclcpp::Node> node_;
    std::unique_ptr<dai::Device> device_;
    std::unique_ptr<dai::rosBridge::ImuConverter> imu_converter_;
    std::unique_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::msg::Imu, dai::IMUData>> imu_publish_;
    std::unique_ptr<dai::rosBridge::ImageConverter> converter_;
    std::unique_ptr<dai::rosBridge::SyncBridgePublisher<sensor_msgs::msg::Image>> cam_publish_;
};

#ifndef OAK_FFC_SYNC_COMPONENT_ONLY
int main(int argc, char** argv)
{

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto oak_node = std::make_shared<OakFfcSyncNode>(options);

    rclcpp::spin(oak_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}
#endif

RCLCPP_COMPONENTS_REGISTER_NODE(OakFfcSyncNode)
