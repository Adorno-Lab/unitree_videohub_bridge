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
 * @file unitree_videohub_bridge_node.cpp
 * @brief Starts UnitreeVideohubBridge: republishes the head camera of a Unitree robot (G1, H1), read through Unitree's
 *        videohub service, on ROS 2. The parameters are documented in UnitreeVideohubBridge.h (set them with the
 *        launch file).
 *
 *        The default SIGINT handler of rclcpp is kept: there is nothing to deinitialize on the robot, so Ctrl+C
 *        stops the spin, and the destructor of the node stops DriverUnitreeVideohub.
 */
#include "UnitreeVideohubBridge.h"

#include <rclcpp/rclcpp.hpp>

#include <iostream>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    int exit_code = 0;
    try
    {
        auto node = std::make_shared<unitree_videohub_bridge::UnitreeVideohubBridge>();
        rclcpp::spin(node);
    }
    catch (const std::exception& e)
    {
        std::cerr << "::Exception::" << e.what() << std::endl;
        exit_code = 1;
    }
    rclcpp::shutdown();
    return exit_code;
}
