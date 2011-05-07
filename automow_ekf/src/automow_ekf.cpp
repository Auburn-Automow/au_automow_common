#include "automow_ekf.h"

#include "math.h"

using namespace automow_ekf;

Automow_EKF::Automow_EKF() {
    // Variable Initialization
    state_estimates << VectorXf::Zero(3), VectorXf::Ones(3), VectorXf::Zero(nx-6);
    estimation_uncertainty << MatrixXf::Identity(nx,nx);
    process_noise << MatrixXf::Identity(nx,nx);
    gps_measurement_noise << MatrixXf::Identity(ny_gps,ny_gps);
    ahrs_measurement_noise << MatrixXf::Identity(ny_ahrs,ny_ahrs);
    previous_input << VectorXf::Zero(nu);
    input_model << MatrixXf::Identity(nx,nx);
    noise_model << MatrixXf::Zero(nx,nx);
}

void Automow_EKF::updateModel(Vector2f input, double delta_time) {
    // Construct the discrete input model (F) from state equations
    input_model(1,3) = -0.5 * delta_time
                       * (state_estimates(4)*input(1) + state_estimates(5)*input(2))
                       * sin(state_estimates(3));
    input_model(1,4) = 0.5 * delta_time * input(1) * cos(state_estimates(3));
    input_model(1,5) = 0.5 * delta_time * input(2) * cos(state_estimates(3));
    input_model(2,3) = 0.5 * delta_time
                       * (state_estimates(4)*input(1) + state_estimates(5)*input(2))
                       * cos(state_estimates(3));
    input_model(2,4) = 0.5 * delta_time * input(1) * sin(state_estimates(3));
    input_model(2,5) = 0.5 * delta_time * input(2) * sin(state_estimates(3));
    input_model(3,4) = -1 * delta_time * (input(1)/state_estimates(6));
    input_model(3,5) = delta_time * (input(1)/state_estimates(6));
    input_model(3,6) = delta_time
                       * ((state_estimates(4)*input(1) - state_estimates(5)*input(2))
                          / (state_estimates(6)^2));
    // Store the input
    this->previous_input = input;
}

void Automow_EKF::timeUpdate(Vector2f input, double current_time) {
    delta_time = current_time - this->previous_time;
    this->previous_time = current_time;
    
    this->updateModel(input, delta_time);
    
    // Calculate linear velocity
    double v = (state_estimates(5)/2.0) * input(2);
    v += (state_estimates(4)/2.0) * input(1);
    // Calculate angular velocity
    double w = (state_estimates(5)/state_estimates(6))*input(2);
    w -= (state_estimates(4)/state_estimates(6))*input(1);
    
    // Update the states based on model and input
    state_estimates(1) += delta_time * v
                          * cos(state_estimates(3) + delta_time * (w/2.0));
    
    state_estimates(2) += delta_time * v
                          * sin(state_estimates(3) + delta_time * (w/2.0));
    state_estimates(3) += delta_time * w;
    estimation_uncertainty = input_model * estimation_uncertainty * input_model.transpose()
                             + process_noise;
}

void Automow_EKF::measurementUpdateGPS(Vector2f measurement, Vector2f covariance) {
    
}

void Automow_EKF::measurementUpdateAHRS() {
    
}




















