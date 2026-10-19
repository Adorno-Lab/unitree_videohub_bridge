/*
#    Copyright (c) 2026 Adorno-Lab
#
#    This is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 2.1 of the License, or
#    (at your option) any later version.
#
#    This software is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with this software. If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: - Juan Jose Quiroz Omana (email: juanjose.quirozomana@manchester.ac.uk)
#           - Documented with the assistance of Claude (Anthropic).
#
# ################################################################
*/

/**
 * @file UnitreeVideohubBridge.cpp
 * @brief Implementation of UnitreeVideohubBridge -- see the header for the overall design.
 */
#include "UnitreeVideohubBridge.h"

#include <unitree/robot/channel/channel_factory.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <chrono>
#include <sstream>
#include <stdexcept>

namespace unitree_videohub_bridge
{
namespace
{
/// Period of the statistics in the log.
constexpr double STATISTICS_PERIOD_SEC = 10.0;
/// Period of the warnings about failed requests.
constexpr double WARNING_PERIOD_SEC = 5.0;
/// Period of the throttled warnings of the capture thread, in milliseconds.
constexpr int WARNING_THROTTLE_MS = 5000;

/**
 * @brief The cv::imdecode flag that decodes a JPEG at 1/downscale of its resolution.
 * @param downscale 1, 2, 4, or 8.
 * @return The flag.
 * @throw std::invalid_argument If downscale is not 1, 2, 4, or 8.
 */
int imdecode_flag(const int downscale)
{
    switch (downscale)
    {
    case 1: return cv::IMREAD_COLOR;
    case 2: return cv::IMREAD_REDUCED_COLOR_2;
    case 4: return cv::IMREAD_REDUCED_COLOR_4;
    case 8: return cv::IMREAD_REDUCED_COLOR_8;
    default:
        throw std::invalid_argument("The parameter downscale must be 1, 2, 4, or 8, but it is " +
                                    std::to_string(downscale) + ".");
    }
}

/**
 * @brief The cv::rotate flag of a clockwise rotation.
 * @param rotation 0, 90, 180, or 270 degrees, clockwise.
 * @return The cv::RotateFlags, or -1 for 0 (no rotation).
 * @throw std::invalid_argument If rotation is not 0, 90, 180, or 270.
 */
int rotate_flag(const int rotation)
{
    switch (rotation)
    {
    case 0:   return -1;
    case 90:  return cv::ROTATE_90_CLOCKWISE;
    case 180: return cv::ROTATE_180;
    case 270: return cv::ROTATE_90_COUNTERCLOCKWISE;
    default:
        throw std::invalid_argument("The parameter rotation must be 0, 90, 180, or 270, but it is " +
                                    std::to_string(rotation) + ".");
    }
}
} // namespace

UnitreeVideohubBridge::UnitreeVideohubBridge(const rclcpp::NodeOptions& options)
    : rclcpp::Node("unitree_videohub_bridge", options)
{
    const int domain_id = declare_parameter<int>("domain_id", 0);
    network_interface_ = declare_parameter<std::string>("network_interface", "eth0");
    service_name_ = declare_parameter<std::string>("service_name", "videohub");
    rate_hz_ = declare_parameter<double>("rate_hz", 30.0);
    rpc_timeout_sec_ = declare_parameter<double>("rpc_timeout_sec", 1.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "head_camera_color_optical_frame");
    topic_ = declare_parameter<std::string>("topic", "/camera/image_raw");
    skip_duplicates_ = declare_parameter<bool>("skip_duplicates", true);
    transcode_ = declare_parameter<bool>("transcode", true);
    downscale_ = declare_parameter<int>("downscale", 2);
    rotation_ = declare_parameter<int>("rotation", 0);

    if (rate_hz_ <= 0.0)
        throw std::invalid_argument("The parameter rate_hz must be positive, but it is " + std::to_string(rate_hz_) + ".");
    imdecode_flag_ = imdecode_flag(downscale_);
    rotate_flag_ = rotate_flag(rotation_);
    if (!transcode_ && rotation_ != 0)
        throw std::invalid_argument("The parameter rotation is " + std::to_string(rotation_) +
                                    ", but without transcode the frames are published as they come and cannot be "
                                    "rotated. Set transcode to true.");

    RCLCPP_INFO_STREAM(get_logger(), "::Parameters OK: domain_id " << domain_id
                                     << ", network_interface " << network_interface_
                                     << ", service_name " << service_name_
                                     << ", rate_hz " << rate_hz_
                                     << ", rpc_timeout_sec " << rpc_timeout_sec_
                                     << ", frame_id " << frame_id_
                                     << ", topic " << topic_
                                     << ", skip_duplicates " << (skip_duplicates_ ? "true" : "false")
                                     << ", transcode " << (transcode_ ? "true" : "false")
                                     << ", downscale " << downscale_
                                     << ", rotation " << rotation_);
    if (!transcode_ && downscale_ != 1)
        RCLCPP_WARN_STREAM(get_logger(), "::downscale " << downscale_
                                         << " is ignored: without transcode, the frames are published as they come.");

    // Reliable: it matches both best-effort subscribers (which then receive best effort) and reliable ones (the
    // default of image_transport, e.g. rqt_image_view and image_transport republish).
    const rclcpp::QoS qos = rclcpp::SensorDataQoS().reliable();
    if (transcode_)
    {
        publisher_image_transport_ = image_transport::create_publisher(this, topic_, qos.get_rmw_qos_profile());
        RCLCPP_INFO_STREAM(get_logger(), "::Publishing the frames of " << service_name_ << " at 1/" << downscale_
                                         << " of their resolution, rotated by " << rotation_ << " degrees, on "
                                         << publisher_image_transport_.getTopic()
                                         << " (all the image_transport plugins, encoded only while subscribed).");
    }
    else
    {
        rclcpp::PublisherOptions publisher_options;
        publisher_options.qos_overriding_options = rclcpp::QosOverridingOptions::with_default_policies();
        publisher_compressed_ = create_publisher<sensor_msgs::msg::CompressedImage>(
            topic_ + "/compressed", qos, publisher_options);
        RCLCPP_INFO_STREAM(get_logger(), "::Publishing the JPEG frames of " << service_name_ << " as they come on "
                                         << publisher_compressed_->get_topic_name());
    }

    // Once per process; the DDS of unitree_sdk2 (Unitree network) is independent of the one of ROS 2.
    unitree::robot::ChannelFactory::Instance()->Init(static_cast<int32_t>(domain_id), network_interface_);
    // Validates rate_hz and rpc_timeout_sec too.
    shutdown_signaler_ = std::make_shared<marinholab::sas::core::ShutdownSignaler>();
    videohub_ = std::make_unique<DriverUnitreeVideohub>(shutdown_signaler_, service_name_, 1.0 / rate_hz_,
                                                        rpc_timeout_sec_, skip_duplicates_);
    videohub_->set_frame_callback([this](const std::shared_ptr<const DriverUnitreeVideohub::Frame>& frame)
                                  { _publish(frame); });

    statistics_start_ = steady_clock_.now();
    statistics_timer_ = create_wall_timer(std::chrono::duration<double>(WARNING_PERIOD_SEC),
                                          [this]() { _check_statistics(); });

    videohub_->connect();
    videohub_->initialize();
}

UnitreeVideohubBridge::~UnitreeVideohubBridge()
{
    // Before the publishers are destroyed: the frame callback uses them.
    shutdown_signaler_->shutdown();
    videohub_->disconnect();
}

void UnitreeVideohubBridge::_publish(const std::shared_ptr<const DriverUnitreeVideohub::Frame>& frame)
{
    if (!rclcpp::ok())
        return;
    const rclcpp::Time stamp = now();

    if (frame->sequence == 1)
    {
        if (frame->width > 0)
            RCLCPP_INFO_STREAM(get_logger(), "::First frame: JPEG " << frame->width << "x" << frame->height << ", "
                                             << frame->jpeg.size() << " bytes.");
        else
            RCLCPP_WARN_STREAM(get_logger(), "::First frame: " << frame->jpeg.size() << " bytes, but not a JPEG image.");
    }

    try
    {
        if (transcode_)
        {
            _publish_transcoded(frame->jpeg, stamp);
            return;
        }

        auto compressed = std::make_unique<sensor_msgs::msg::CompressedImage>();
        compressed->header.stamp = stamp;
        compressed->header.frame_id = frame_id_;
        compressed->format = "jpeg";
        compressed->data = frame->jpeg;
        publisher_compressed_->publish(std::move(compressed));
        std::scoped_lock lock(counters_mutex_);
        counters_.published++;
    }
    catch (const std::exception& e)
    {
        // E.g. the context was shut down between the check and the publication.
        if (rclcpp::ok())
            RCLCPP_ERROR_STREAM_THROTTLE(get_logger(), steady_clock_, WARNING_THROTTLE_MS, "::Exception::" << e.what());
    }
}

void UnitreeVideohubBridge::_publish_transcoded(const std::vector<uint8_t>& jpeg, const rclcpp::Time& stamp)
{
    // Nobody listens on any transport: skip the decoding too.
    if (publisher_image_transport_.getNumSubscribers() == 0)
    {
        std::scoped_lock lock(counters_mutex_);
        counters_.without_subscribers++;
        return;
    }

    const auto start = std::chrono::steady_clock::now();
    cv::Mat decoded = cv::imdecode(jpeg, imdecode_flag_);
    const double decode_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    {
        std::scoped_lock lock(counters_mutex_);
        counters_.decode_total_ms += decode_ms;
        counters_.decodes++;
    }
    if (decoded.empty())
    {
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(), steady_clock_, WARNING_THROTTLE_MS,
                                    "::Could not decode a frame of " << jpeg.size() << " bytes.");
        return;
    }
    // H.264 (yuv420p) needs even sizes, e.g. 1080/8 = 135 rows: drop the last row/column (a view, no copy). Done
    // before the rotation, which keeps the sizes even.
    decoded = decoded(cv::Rect(0, 0, decoded.cols & ~1, decoded.rows & ~1));

    // A quarter turn swaps the width and the height.
    const bool quarter_turn = rotation_ == 90 || rotation_ == 270;
    auto image = std::make_unique<sensor_msgs::msg::Image>();
    image->header.stamp = stamp;
    image->header.frame_id = frame_id_;
    image->height = static_cast<uint32_t>(quarter_turn ? decoded.cols : decoded.rows);
    image->width = static_cast<uint32_t>(quarter_turn ? decoded.rows : decoded.cols);
    image->encoding = "bgr8";
    image->is_bigendian = false;
    image->step = static_cast<uint32_t>(image->width * decoded.elemSize());
    image->data.resize(static_cast<std::size_t>(image->step) * image->height);

    // Rotate (or copy) straight into the data of the message: a cv::Mat that wraps it, with the same size and type,
    // so OpenCV writes into it instead of allocating.
    cv::Mat output(static_cast<int>(image->height), static_cast<int>(image->width), decoded.type(),
                   image->data.data(), image->step);
    if (rotate_flag_ < 0)
        decoded.copyTo(output);
    else
        cv::rotate(decoded, output, rotate_flag_);
    if (output.data != image->data.data())
        throw std::runtime_error("OpenCV reallocated the output image instead of writing into the message.");

    publisher_image_transport_.publish(std::move(image));
    std::scoped_lock lock(counters_mutex_);
    counters_.published++;
}

void UnitreeVideohubBridge::_check_statistics()
{
    const DriverUnitreeVideohub::Statistics statistics = videohub_->get_statistics();
    PublicationCounters counters;
    {
        std::scoped_lock lock(counters_mutex_);
        counters = counters_;
    }

    if (statistics.failed_requests > warned_failures_)
    {
        RCLCPP_WARN_STREAM(get_logger(), "::" << statistics.failed_requests - warned_failures_ << " requests to "
                                         << service_name_ << " failed in the last " << WARNING_PERIOD_SEC
                                         << " s, the last one with code " << statistics.last_error_code << " ("
                                         << DriverUnitreeVideohub::get_error_description(statistics.last_error_code)
                                         << ").");
        warned_failures_ = statistics.failed_requests;
    }

    const rclcpp::Time now_steady = steady_clock_.now();
    const double elapsed_sec = (now_steady - statistics_start_).seconds();
    // Half a timer period of margin: the timer can fire slightly early (e.g. after 9.9999 s).
    if (elapsed_sec < STATISTICS_PERIOD_SEC - 0.5 * WARNING_PERIOD_SEC)
        return;

    const uint64_t requests = statistics.requests - last_statistics_.requests;
    const double request_ms = statistics.total_request_duration_ms - last_statistics_.total_request_duration_ms;
    const uint64_t published = counters.published - last_counters_.published;
    const uint64_t decodes = counters.decodes - last_counters_.decodes;
    const double decode_ms = counters.decode_total_ms - last_counters_.decode_total_ms;
    std::ostringstream decoding;
    if (transcode_)
        decoding << ", " << counters.without_subscribers - last_counters_.without_subscribers
                 << " not decoded (no subscribers), mean decoding "
                 << (decodes > 0 ? decode_ms / static_cast<double>(decodes) : 0.0) << " ms";
    RCLCPP_INFO_STREAM(get_logger(), "::Last " << elapsed_sec << " s: published " << published << " frames ("
                                     << published / elapsed_sec << " Hz), "
                                     << statistics.duplicate_frames - last_statistics_.duplicate_frames
                                     << " duplicates skipped, "
                                     << statistics.failed_requests - last_statistics_.failed_requests
                                     << " failed requests, mean request "
                                     << (requests > 0 ? request_ms / static_cast<double>(requests) : 0.0)
                                     << " ms" << decoding.str() << ".");
    last_statistics_ = statistics;
    last_counters_ = counters;
    statistics_start_ = now_steady;
}
} // namespace unitree_videohub_bridge
