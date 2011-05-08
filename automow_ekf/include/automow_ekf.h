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

#include <Eigen/Eigen>
using Eigen::Matrix;
using Eigen::MatrixXf;
using Eigen::Vector2f;
using Eigen::Vector3f;

namespace automow_ekf {

const int nx = 9;
const int ny_gps = 2;
const int ny_ahrs = 1;
const int nu = 2;

Matrix<float, ny_gps, nx> gps_measurement_model = 
    (MatrixXf(ny_gps, nx) << 0,1,0,0,0,0,0,0,1, 1,0,0,0,0,0,0,1,0
    ).finished();

Matrix<float, ny_ahrs, nx> ahrs_measurement_model = 
    (MatrixXf(ny_ahrs, nx) << 0,0,1,0,0,0,0,0,1).finished();

class Automow_EKF {
public:
    /** Constructor */
    Automow_EKF();
    
    /** Destructor */
    ~Automow_EKF();
    
    void timeUpdate(double left_wheel, double right_wheel, double current_time);
    
    void measurementUpdateGPS(double northing, double easting, double northing_covariance, double easting_covariance);
    
    void measurementUpdateAHRS(float measurement, float covariance);
    
    double getNorthing();
    
    double getEasting();
    
    double getYaw();
    
private:
    void updateModel(Vector2f input, double delta_time);
    
    Matrix<float, nx, 1> state_estimates; // x_hat
    Matrix<float, nx, nx> estimation_uncertainty; // P
    Matrix<float, nx, nx> process_noise; // Q
    Matrix<float, ny_gps, ny_gps> gps_measurement_noise; // R_gps
    Matrix<float, ny_ahrs, ny_ahrs> ahrs_measurement_noise; // R_imu
    Matrix<float, nu, 1> previous_input; // prev_u
    double previous_time; // prev_time
    Matrix<float, nx, nx> input_model; // F
    Matrix<float, nx, nx> noise_model; // G
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