/**
 * @file automow_ekf.h
 * @author  William Woodall <wjwwood@gmail.com>
 * @author  Michael Carroll <carroll.michael@gmail.com>
 * @version 0.1
 *
 * @section LICENSE
 *
 * The MIT License
 *
 * Copyright (c) 2011 William Woodall, Michael Carroll
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * @section DESCRIPTION
 *
 * This is an implementation of an 8 state Kalman Filter for the Autonomous Lawnmower.
 */


#ifndef AUTOMOW_EKF_H
#define AUTOMOW_EKF_H

#include <iostream>
#include <sstream>
#include <string>

#include <Eigen/Dense>
using Eigen::MatrixXd;

namespace automow_ekf {

static int nx = 9;
static int ny_gps = 2;
static int ny_ahrs = 1;
static int nu = 2;

static Matrix<float, nx, ny_gps> 
gps_measurement_model = 
    (MatrixXf(nx, ny_gps) << MatrixXf::Identity(ny_gps,ny_gps),
                             MatrixXf::Zero(nx-ny_gps, ny_gps),
                             MatrixXf::Identity(ny_gps,ny_gps)
    ).finished();

static Matrix<float, nx, ny_ahrs> 
ahrs_measurement_noise = 
    (MatrixXf(nx, ny_ahrs) << 0,0,1,0,0,0,1,0,0).finished();

class Automow_EKF {
public:
    /** Constructor */
    Automow_EKF();
    
    /** Destructor */
    ~Automow_EKF();
    
    void updateModel(Vector2f input, double delta_time);
    
    void timeUpdate(Vector2f input, double delta_time);
    
    void measurementUpdateGPS();
    
    void measurementUpdateAHRS();
    
private:
    Vector<float, nx> state_estimates; // x_hat
    Matrix<float, nx, nx> estimation_uncertainty; // P
    Matrix<float, nx, nx> process_noise; // Q
    Matrix<float, ny_gps, ny_gps> gps_measurement_noise; // R_gps
    Matrix<float, ny_ahrs, my_ahrs> ahrs_measurement_noise; // R_imu
    vector<float, nu> previouse_input; // prev_u
    double previous_time; // prev_time
    Matrix<float, nx, nx> input_model; // F
    Matrix<float, nx, nx> noise_model; // G
    Matrix
};

// class SerialPortAlreadyOpenException : public std::exception {
//     const char * port;
// public:
//     SerialPortAlreadyOpenException(const char * port) {this->port = port;}
//     
//     virtual const char* what() const throw() {
//         std::stringstream ss;
//         ss << "Serial Port already open: " << this->port;
//         return ss.str().c_str();
//     }
// };

} // namespace automow_ekf

#endif