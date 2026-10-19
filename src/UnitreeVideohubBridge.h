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
 * @file UnitreeVideohubBridge.h
 * @brief ROS 2 node that republishes the head camera of a Unitree robot (G1, H1), read through Unitree's videohub
 *        service with DriverUnitreeVideohub (unitree_drivers), without opening the camera device that videohub owns.
 *
 *        DriverUnitreeVideohub requests the frames in its own thread, at up to rate_hz, skips the duplicates, and
 *        calls this node with each new frame (a JPEG). Each new frame is published in one of two modes:
 *
 *          - transcode (default): the JPEG is decoded directly at a reduced resolution (1/downscale; libjpeg scales
 *            in the DCT, which is cheaper than decoding the full frame and resizing it), rotated by rotation degrees
 *            (the H1 camera is mounted upside down), and published with an image_transport publisher on `<topic>`.
 *            Every installed transport plugin is then available: ffmpeg (H.264, the least bandwidth), compressed
 *            (JPEG), and raw. A plugin encodes only while its topic has subscribers, and the frame is not decoded at
 *            all while no transport has subscribers.
 *          - passthrough (transcode false): the JPEG of videohub is published as it comes, at full resolution, as
 *            sensor_msgs/CompressedImage on `<topic>/compressed` (the topic of the "compressed" plugin). No decoding on
 *            the robot, but ~145 KB per frame. It cannot be rotated.
 */
#pragma once

#include <unitree_drivers/DriverUnitreeVideohub.h>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <marinholab/sas/core/sas_shutdown_signaler.hpp>

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace unitree_videohub_bridge
{
/**
 * @brief Republishes the frames of videohub on ROS 2.
 *
 * @details The modes of publication are described in the documentation of the file (UnitreeVideohubBridge.h).
 * ROS parameters of the node:
 *
 * | Parameter | Type | Default | Description |
 * |---|---|---|---|
 * | `domain_id` | int | `0` | DDS domain of the Unitree network (0 on the robot) |
 * | `network_interface` | string | `"eth0"` | interface on the Unitree network (192.168.123.x) |
 * | `service_name` | string | `"videohub"` | name of the videohub RPC service |
 * | `rate_hz` | double | `30.0` | maximum rate of the requests to videohub |
 * | `rpc_timeout_sec` | double | `1.0` | timeout of each request |
 * | `frame_id` | string | `"head_camera_color_optical_frame"` | frame_id of the published images |
 * | `topic` | string | `"/camera/image_raw"` | base topic of the images |
 * | `skip_duplicates` | bool | `true` | do not publish a frame identical to the previous one |
 * | `transcode` | bool | `true` | decode, downscale, rotate, and publish through image_transport; false publishes the JPEG of videohub as it comes |
 * | `downscale` | int | `2` | 1, 2, 4, or 8: the resolution is divided by it when transcoding (1920x1080 to 960x540, 480x270, 240x134), rounded down to even sizes, as H.264 (yuv420p) needs them |
 * | `rotation` | int | `0` | 0, 90, 180, or 270: clockwise rotation of the image, in degrees, when transcoding (180 on the H1, whose camera is upside down); must be 0 without transcode |
 *
 * The parameters of the transport plugins follow the image_transport conventions, e.g.
 * `camera.image_raw.ffmpeg.encoder` or `camera.image_raw.compressed.jpeg_quality` (see
 * config/unitree_videohub_bridge.yaml).
 *
 * The QoS is the sensor-data profile, but reliable, so that both best-effort and reliable subscribers match.
 */
class UnitreeVideohubBridge : public rclcpp::Node
{
public:
    /**
     * @brief Reads the parameters, initializes the Unitree channel factory, creates the publishers, and starts
     *        DriverUnitreeVideohub.
     * @param options Options of the node.
     * @throw std::invalid_argument If a parameter is invalid.
     */
    explicit UnitreeVideohubBridge(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

    /**
     * @brief Stops DriverUnitreeVideohub (it can take up to rpc_timeout_sec, if a request is pending).
     */
    ~UnitreeVideohubBridge() override;

private:
    /**
     * @brief Publishes a new frame, in the mode set by the parameter transcode. Called by DriverUnitreeVideohub, on
     *        its capture thread.
     * @param frame The new frame.
     */
    void _publish(const std::shared_ptr<const DriverUnitreeVideohub::Frame>& frame);

    /**
     * @brief Decodes a frame at 1/downscale of its resolution, rotates it, and publishes it through image_transport.
     *        Does nothing while no transport has subscribers.
     * @param jpeg The JPEG bytes of the frame.
     * @param stamp The time at which the frame was received.
     */
    void _publish_transcoded(const std::vector<uint8_t>& jpeg, const rclcpp::Time& stamp);

    /**
     * @brief Called by a timer every WARNING_PERIOD_SEC: warns about the requests that failed since the last call,
     *        and logs the statistics every STATISTICS_PERIOD_SEC (frames published, duplicates, failures, duration of
     *        the requests and decoding).
     */
    void _check_statistics();

    // Parameters.
    std::string network_interface_;
    std::string service_name_;
    double rate_hz_;
    double rpc_timeout_sec_;
    std::string frame_id_;
    std::string topic_;
    bool skip_duplicates_;
    bool transcode_;
    int downscale_;
    int rotation_;

    /// cv::imdecode flag that decodes at 1/downscale_ of the resolution.
    int imdecode_flag_;
    /// cv::RotateFlags for rotation_, or -1 for no rotation.
    int rotate_flag_;

    std::shared_ptr<marinholab::sas::core::ShutdownSignaler> shutdown_signaler_;
    std::unique_ptr<DriverUnitreeVideohub> videohub_;

    /// Passthrough mode.
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher_compressed_;
    /// Transcode mode.
    image_transport::Publisher publisher_image_transport_;

    rclcpp::TimerBase::SharedPtr statistics_timer_;
    /// Steady clock for the throttled logs (the clock of the onboard computer may be far from the real time).
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    /// Counters of the publication, written on the capture thread and read by the statistics timer.
    struct PublicationCounters
    {
        uint64_t published{0};
        uint64_t without_subscribers{0};
        uint64_t decodes{0};
        double decode_total_ms{0.0};
    };
    std::mutex counters_mutex_;
    PublicationCounters counters_;

    /// Readings at the start of the current statistics period (statistics timer only).
    DriverUnitreeVideohub::Statistics last_statistics_;
    PublicationCounters last_counters_;
    rclcpp::Time statistics_start_;
    /// Failed requests already warned about (statistics timer only).
    uint64_t warned_failures_{0};
};
} // namespace unitree_videohub_bridge
