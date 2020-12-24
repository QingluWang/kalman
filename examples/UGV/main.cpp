
// this MUST be first, otherwise there might be problems on windows
// see: https://stackoverflow.com/questions/6563810/m-pi-works-with-math-h-but-not-with-cmath-in-visual-studio/6563891#6563891
#define _USE_MATH_DEFINES
#include <cmath>


#include "SystemModel.hpp"
#include "OrientationMeasurementModel.hpp"
#include "PositionMeasurementModel.hpp"

#include <kalman/ExtendedKalmanFilter.hpp>
#include <kalman/UnscentedKalmanFilter.hpp>

#include <iostream>
#include <random>
#include <chrono>
#include <fstream>

using namespace KalmanExamples;

typedef float T;

// Some type shortcuts
typedef UGV::State<T> State;
typedef UGV::Control<T> Control;
typedef UGV::SystemModel<T> SystemModel;

typedef UGV::PositionMeasurement<T> PositionMeasurement;
typedef UGV::OrientationMeasurement<T> OrientationMeasurement;
typedef UGV::PositionMeasurementModel<T> PositionModel;
typedef UGV::OrientationMeasurementModel<T> OrientationModel;

int main(int argc, char** argv)
{
    // Simulated (true) system state
    State x;
    x.setZero();
    
    // Control input
    Control u;
    // System
    SystemModel sys;
    
    // Measurement models
    PositionModel pm(-10,-10,0,30,20,0);
    OrientationModel om;
    
    // Random number generation (for noise simulation)
    std::default_random_engine generator;
    generator.seed( std::chrono::system_clock::now().time_since_epoch().count() );
    std::normal_distribution<T> noise(0, 1);
    
    // Some filters for estimation
    // Pure predictor without measurement updates
    Kalman::ExtendedKalmanFilter<State> predictor;
    // Extended Kalman Filter
    Kalman::ExtendedKalmanFilter<State> ekf;
    // Unscented Kalman Filter
    Kalman::UnscentedKalmanFilter<State> ukf(1);
    
    // Init filters with true system state
    predictor.init(x);
    ekf.init(x);
    ukf.init(x);
    
    // Standard-Deviation of noise added to all state vector components during state transition
    T systemNoise = 0.1;
    // Standard-Deviation of noise added to all measurement vector components in orientation measurements
    T orientationNoise = 0.025;
    // Standard-Deviation of noise added to all measurement vector components in distance measurements
    T distanceNoise = 0.25;
    
    // output file stream
    std::ofstream output_fstream("../examples/UGV/data.csv");
    if(!output_fstream.is_open())
        return 0;
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        // Generate some control input
        u.a_x() = 1.01;
        u.a_y() = 1.02;
        u.a_z() = 0;
        u.omega_x() = 0.2;
        u.omega_y() = 0.2;
        u.omega_z() = 0;
        
        // Simulate system
        x = sys.f(x, u);
        
        // Add noise: Our robot move is affected by noise (due to actuator failures)
        x.p_x() += systemNoise*noise(generator);
        x.p_y() += systemNoise*noise(generator);
        x.p_z() += systemNoise*noise(generator);
        x.v_x() += systemNoise*noise(generator);
        x.v_y() += systemNoise*noise(generator);
        x.v_z() += systemNoise*noise(generator);
        x.theta_r() += systemNoise*noise(generator);
        x.theta_p() += systemNoise*noise(generator);
        x.theta_y() += systemNoise*noise(generator);
        x.omega_x() += systemNoise*noise(generator);
        x.omega_y() += systemNoise*noise(generator);
        x.omega_z() += systemNoise*noise(generator);
        
        // Predict state for current time-step using the filters
        auto x_pred = predictor.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);
        
        // Orientation measurement
        {
            // We can measure the orientation every 5th step
            OrientationMeasurement orientation = om.h(x);
            
            // Measurement is affected by noise as well
            orientation.theta_r() += orientationNoise * noise(generator);
            orientation.theta_p() += orientationNoise * noise(generator);
            orientation.theta_y() += orientationNoise * noise(generator);
            

            // Update UKF
            x_ukf = ukf.update(om, orientation);
        }
        
        // Position measurement
        {
            // We can measure the position every 10th step
            PositionMeasurement position = pm.h(x);
            
            // Measurement is affected by noise as well
            position.p_x() += distanceNoise * noise(generator);
            position.p_y() += distanceNoise * noise(generator);
            

            // Update UKF
            x_ukf = ukf.update(pm, position);
        }
        
        // Print to stdout as csv format
        // std::cout   << x.p_x() << "," << x.p_y() << "," << x.p_z() << "," << x.theta_r() << ","<< x.theta_p() << ","<< x.theta_y() << ","
        //             << x_pred.p_x() << "," << x_pred.p_y() << "," << x_pred.p_z() << "," << x_pred.theta_r() << ","<< x_pred.theta_p() << ","<< x_pred.theta_y() << ","
        //             << x_ukf.p_x() << "," << x_ukf.p_y() << "," << x_ukf.p_z() << "," << x_ukf.theta_r() << ","<< x_ukf.theta_p() << ","<< x_ukf.theta_y()
        //             << std::endl;
        output_fstream  << x.p_x() << "," << x.p_y()  << "," << x.theta_r() << ","<< x.theta_p() << ","
                        << x_pred.p_x() << "," << x_pred.p_y() << "," << x_pred.theta_r() << ","<< x_pred.theta_p() << ","
                        << x_ukf.p_x() << "," << x_ukf.p_y() << "," << x_ukf.theta_r() << ","<< x_ukf.theta_p()
                        << std::endl;
    }
    output_fstream.close();
    return 0;
}
