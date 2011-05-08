#include "automow_ekf.h"

#include "math.h"

using namespace automow_ekf;

Automow_EKF::Automow_EKF() {
    // Variable Initialization
    state_estimates << Vector3f::Zero(3), Vector3f::Ones(3), Vector3f::Zero(3);
    estimation_uncertainty << MatrixXf::Identity(nx,nx);
    process_noise << MatrixXf::Identity(nx,nx);
    gps_measurement_noise << MatrixXf::Identity(ny_gps,ny_gps);
    ahrs_measurement_noise << MatrixXf::Identity(ny_ahrs,ny_ahrs);
    previous_input << Vector2f::Zero(nu);
    input_model << MatrixXf::Identity(nx,nx);
    noise_model << MatrixXf::Zero(nx,nx);
}

Automow_EKF::~Automow_EKF() {
    
}

void Automow_EKF::timeUpdate(double left_wheel, double right_wheel, double current_time) {
    double delta_time = current_time - this->previous_time;
    this->previous_time = current_time;
    
    Vector2f input(left_wheel, right_wheel);
    
    this->updateModel(input, delta_time);
    
    // Calculate linear velocity
    double v = (state_estimates(4)/2.0) * input(1);
    v += (state_estimates(3)/2.0) * input(0);
    // Calculate angular velocity
    double w = (state_estimates(4)/state_estimates(5))*input(1);
    w -= (state_estimates(3)/state_estimates(5))*input(0);
    
    // Update the states based on model and input
    state_estimates(0) += delta_time * v
                          * cos(state_estimates(2) + delta_time * (w/2.0));
    
    state_estimates(1) += delta_time * v
                          * sin(state_estimates(2) + delta_time * (w/2.0));
    state_estimates(2) += delta_time * w;
    estimation_uncertainty = input_model * estimation_uncertainty * input_model.transpose()
                             + process_noise;
}

void Automow_EKF::measurementUpdateGPS(double northing, double easting, double northing_covariance, double easting_covariance) {
    Vector2f measurement(easting, northing);
    Vector2f covariance(easting_covariance, northing_covariance);
    Vector2f innovation = measurement - (gps_measurement_model * state_estimates);
    Matrix<float, nx, 2> kalman_gain;
    Matrix<float, 2, 2> S;
    S = gps_measurement_model * estimation_uncertainty * gps_measurement_model.transpose();
    S += covariance.asDiagonal();
    kalman_gain = estimation_uncertainty * gps_measurement_model.transpose() * S.inverse();
    state_estimates += kalman_gain * innovation;
    estimation_uncertainty *= (MatrixXf::Identity(nx,nx) - kalman_gain*gps_measurement_model);
}

void Automow_EKF::measurementUpdateAHRS(float measurement, float covariance) {
    Matrix<float, 1, 1> innovation = Matrix<float, 1, 1>::Constant(measurement) - (ahrs_measurement_model * state_estimates);
    Matrix<float, nx, 1> kalman_gain;
    Matrix<float, 1, 1> S;
    S = ahrs_measurement_model * estimation_uncertainty * ahrs_measurement_model.transpose();
    S += Matrix<float, 1, 1>::Constant(covariance);
    kalman_gain = (estimation_uncertainty * ahrs_measurement_model.transpose()) * S.inverse();
    state_estimates += kalman_gain * innovation;
    estimation_uncertainty *= (MatrixXf::Identity(nx,nx) - kalman_gain*ahrs_measurement_model);
}

double Automow_EKF::getNorthing() {
    return this->state_estimates(0);
}

double Automow_EKF::getEasting() {
    return this->state_estimates(1);
}

double Automow_EKF::getYaw() {
    return this->state_estimates(2);
}

void Automow_EKF::updateModel(Vector2f input, double delta_time) {
    // Construct the discrete input model (F) from state equations
    input_model(0,2) = -0.5 * delta_time
                       * (state_estimates(3)*input(0) + state_estimates(4)*input(1))
                       * sin(state_estimates(2));
    input_model(0,3) = 0.5 * delta_time * input(0) * cos(state_estimates(2));
    input_model(0,4) = 0.5 * delta_time * input(1) * cos(state_estimates(2));
    input_model(1,2) = 0.5 * delta_time
                       * (state_estimates(3)*input(0) + state_estimates(4)*input(1))
                       * cos(state_estimates(2));
    input_model(1,3) = 0.5 * delta_time * input(0) * sin(state_estimates(2));
    input_model(1,4) = 0.5 * delta_time * input(1) * sin(state_estimates(2));
    input_model(2,3) = -1 * delta_time * (input(0)/state_estimates(5));
    input_model(2,4) = delta_time * (input(1)/state_estimates(5));
    input_model(2,5) = delta_time
                       * ((state_estimates(3)*input(0) - state_estimates(4)*input(1))
                          / pow(state_estimates(5),2));
    
    // Construct the noise model (G) from state equations
    noise_model(0,0) = 0.5 * delta_time * state_estimates(3) * cos(state_estimates(2));
    noise_model(0,1) = 0.5 * delta_time * state_estimates(4) * cos(state_estimates(2));
    noise_model(0,3) = 0.5 * delta_time * input(0) * cos(state_estimates(2));
    noise_model(0,4) = 0.5 * delta_time * input(1) * cos(state_estimates(2));
    noise_model(1,0) = 0.5 * delta_time * state_estimates(3) * sin(state_estimates(2));
    noise_model(1,1) = 0.5 * delta_time * state_estimates(4) * sin(state_estimates(2));
    noise_model(1,3) = 0.5 * delta_time * input(0) * cos(state_estimates(2));
    noise_model(1,4) = 0.5 * delta_time * input(1) * cos(state_estimates(2));
    noise_model(2,0) = -1*delta_time * state_estimates(3)/state_estimates(5);
    noise_model(2,1) = delta_time * state_estimates(4)/state_estimates(5);
    noise_model(2,2) = delta_time;
    noise_model(2,3) = -1*delta_time * state_estimates(3)/state_estimates(5);
    noise_model(2,4) = delta_time * state_estimates(4)/state_estimates(5);
    noise_model(3,3) = delta_time;
    noise_model(4,4) = delta_time;
    noise_model(5,5) = delta_time;
    noise_model(6,6) = delta_time;
    noise_model(7,7) = delta_time;
    noise_model(8,8) = delta_time;
    
    // Store the input
    this->previous_input = input;
}


















