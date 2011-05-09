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
using Eigen::MatrixXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;

namespace automow_ekf {

const int nx = 9;
const int ny_gps = 2;
const int ny_ahrs = 1;
const int nu = 2;

Matrix<double, ny_gps, nx> gps_measurement_model = 
    (MatrixXd(ny_gps, nx) << 1,0,0,0,0,0,1,0,0, 0,1,0,0,0,0,0,1,0
    ).finished();

Matrix<double, ny_ahrs, nx> ahrs_measurement_model = 
    (MatrixXd(ny_ahrs, nx) << 0,0,1,0,0,0,0,0,0).finished();

class Automow_EKF {
public:
    /** Constructor */
    Automow_EKF();
    
    /** Destructor */
    ~Automow_EKF();
    
    void timeUpdate(double left_wheel, double right_wheel, double current_time);
    
    void measurementUpdateGPS(double northing, double easting, double northing_covariance, double easting_covariance);
    
    void measurementUpdateAHRS(double measurement, double covariance);
    
    double getNorthing();
    
    double getEasting();
    
    double getYaw();
    
private:
    void updateModel(Vector2d input, double delta_time);
    double wrapToPi(double angle);
    
    Matrix<double, nx, 1> state_estimates; // x_hat
    Matrix<double, nx, nx> estimation_uncertainty; // P
    Matrix<double, nx, nx> process_noise; // Q
    Matrix<double, ny_gps, ny_gps> gps_measurement_noise; // R_gps
    Matrix<double, ny_ahrs, ny_ahrs> ahrs_measurement_noise; // R_imu
    double previous_time; // prev_time
    Matrix<double, nx, nx> input_model; // F
    Matrix<double, nx, nx> noise_model; // G
    bool model_initialized;
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