/**
 * MIT License
 *
 * Copyright (c) 2016 Patrick Geneva <pgeneva@udel.edu>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "kitti_parser/Parser.h"
#include <kitti_parser/types/stereo_t.h>
#include <kitti_parser/types/lidar_t.h>
#include <kitti_parser/types/gpsimu_t.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <iostream>

using namespace std;
using namespace kitti_parser;


/**
 * Default constructor
 * This should check to make sure that the path is valid
 * Also make sure we have the configuration information
 * Can preload all the details, so the run function can run
 */
Parser::Parser(std::string path, DataSetFilter filter) {

    // Save with a slash
    // http://stackoverflow.com/a/4884579
    if (*path.rbegin() == '/') {
        config.path_data = path;
    } else {
        config.path_data = path + "/";
    }

    // Check if directory
    if (!std::filesystem::exists(config.path_data)) {
        std::cerr << "[kitti_parser]: Unable to open path" << std::endl;
        return;
    }

    // Set the configuration files
    config.path_calib_cc = config.path_data + "calib_cam_to_cam.txt";
    config.path_calib_iv = config.path_data + "calib_imu_to_velo.txt";
    config.path_calib_vc = config.path_data + "calib_velo_to_cam.txt";

    // Check to see if configuration file CAM to CAM
    if (std::filesystem::exists(config.path_calib_cc)) {
        // Load in the config file
        YAML::Node temp = YAML::LoadFile(config.path_calib_cc);
        // Loop through nodes, and load each one into a double array
        for (YAML::const_iterator it = temp.begin(); it != temp.end(); ++it) {
            // Debug
            //std::cout << "Node: " << it->first.as<std::string>() << " - val - " << it->second.as<std::string>() << "\n";
            // Create node
            std::string str;
            stringstream s(it->second.as<std::string>());
            while (s >> str) {
                double val = (double) stod(str.c_str());
                config.calib_cc[it->first.as<std::string>()].push_back(val);
            }

        }
        // Set enabled
        config.has_calib_cc = true;
    }

    // Check to see if configuration file IMU to VELO
    if (std::filesystem::exists(config.path_calib_iv)) {
        // Load in the config file
        YAML::Node temp = YAML::LoadFile(config.path_calib_iv);
        // Loop through nodes, and load each one into a double array
        for (YAML::const_iterator it = temp.begin(); it != temp.end(); ++it) {
            // Debug
            //std::cout << "Node: " << it->first.as<std::string>() << " - val - " << it->second.as<std::string>() << "\n";
            // Create node
            std::string str;
            stringstream s(it->second.as<std::string>());
            while (s >> str) {
                double val = (double) stod(str.c_str());
                config.calib_iv[it->first.as<std::string>()].push_back(val);
            }

        }
        // Set enabled
        config.has_calib_iv = true;
    }

    // Check to see if configuration file VELO to CAM
    if (std::filesystem::exists(config.path_calib_vc)) {
        // Load in the config file
        YAML::Node temp = YAML::LoadFile(config.path_calib_vc);
        // Loop through nodes, and load each one into a double array
        for (YAML::const_iterator it = temp.begin(); it != temp.end(); ++it) {
            // Debug
            //std::cout << "Node: " << it->first.as<std::string>() << " - val - " << it->second.as<std::string>() << "\n";
            // Create node
            std::string str;
            stringstream s(it->second.as<std::string>());
            while (s >> str) {
                double val = (double) stod(str.c_str());
                config.calib_vc[it->first.as<std::string>()].push_back(val);
            }

        }
        // Set enabled
        config.has_calib_vc = true;
    }

    // Create our loader
    loader = new Loader(&config);


    // Loop through all sub folders, assume they are sequential
    // http://www.boost.org/doc/libs/1_47_0/libs/filesystem/v3/example/tut4.cpp
    std::filesystem::path p(config.path_data);
    vector<std::filesystem::path> v;
    copy(std::filesystem::directory_iterator(p), std::filesystem::directory_iterator(), back_inserter(v));

    // Sort, since directory iteration
    // Is not ordered on some file systems
    sort(v.begin(), v.end());


    for (vector<std::filesystem::path>::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it) {

        // Skip if file
        if (!std::filesystem::is_directory(*it))
            continue;

        if (!filter(*it)) {
            std::cout << "Skipping filtered data set: " << it->filename() << std::endl;
            continue;
        }

        // Next sub folder
        string subfolder = (*it).c_str();
        //std::cout << (*it) << "\n";

        // Check to see is gray camera is there
        if (std::filesystem::exists(subfolder + "/image_00/")
            && std::filesystem::exists(subfolder + "/image_01/")) {
            config.has_stereo_gray = true;
        }

        // Check to see if color camera is there
        if (std::filesystem::exists(subfolder + "/image_02/")
            && std::filesystem::exists(subfolder + "/image_03/")) {
            config.has_stereo_color = true;
        }

        // Check to see if lidar is there
        if (std::filesystem::exists(subfolder + "/velodyne_points/")) {
            config.has_lidar = true;
        }

        // Check to see if IMU is there
        if (std::filesystem::exists(subfolder + "/oxts/")) {
            config.has_gpsimu = true;
        }

        // Done, load all the data
        loader->load_all(subfolder);
    }

}


/**
 * Returns the current config file
 */
Config Parser::getConfig() {
    return config;
}


void Parser::register_callback_stereo_gray(std::function<void(Config *, long, stereo_t *)> callback) {
    callback_stereo_gray = callback;
}

void Parser::register_callback_stereo_color(std::function<void(Config *, long, stereo_t *)> callback) {
    callback_stereo_color = callback;
}

void Parser::register_callback_lidar(std::function<void(Config *, long, lidar_t *)> callback) {
    callback_lidar = callback;
}

void Parser::register_callback_gpsimu(std::function<void(Config *, long, gpsimu_t *)> callback) {
    callback_gpsimu = callback;
}

/**
 * This function will call the callback functions and pass the data
 * This can be run at double the speed, but will be limited by
 * how fast the program can process, as this is not multi-threaded
 */
void Parser::run(double time_multi) {

    // Load the first message
    Loader::message_types *next = loader->fetch_latest();

    // Loop till we run out of message to send
    // http://stackoverflow.com/a/5685578
    while (next != nullptr) {

        // Call the respective callbacks based on that type
        struct Visitor {
            Parser &parser;

            void operator()(stereo_t *temp_s) {
                // Send, and check if valid function
                if (temp_s->is_color && parser.callback_stereo_color) {
                    parser.callback_stereo_color.operator()(&parser.config, temp_s->timestamp, temp_s);
                }
                    // Check if function has been set
                else if (!temp_s->is_color && parser.callback_stereo_gray) {
                    parser.callback_stereo_gray.operator()(&parser.config, temp_s->timestamp, temp_s);
                }
                    // Else free it since nobody wants it
                else {
                    delete temp_s;
                }
            }

            void operator()(lidar_t *temp_v) {
                // Check if function has been set
                if (parser.callback_lidar) {
                    parser.callback_lidar.operator()(&parser.config, temp_v->timestamp, temp_v);
                } else {
                    delete temp_v;
                }
            }

            void operator()(gpsimu_t *temp_g) {
                // Check if function has been set
                if (parser.callback_gpsimu) {
                    parser.callback_gpsimu.operator()(&parser.config, temp_g->timestamp, temp_g);
                } else {
                    delete temp_g;
                }
            };
        };
        Visitor visitor{*this};
        std::visit(visitor, *next);


        // Get the next message from the loader
        next = loader->fetch_latest();
    }

}

