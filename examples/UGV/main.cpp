
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
    PositionModel pm;
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
    
    // Simulate for 100 steps
    const size_t N = 100;
    for(size_t i = 1; i <= N; i++)
    {
        // Generate some control input
        u.v() = 1. + std::sin( T(2) * T(M_PI) / T(N) );
        u.dtheta() = std::sin( T(2) * T(M_PI) / T(N) ) * (1 - 2*(i > 50));
        
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
        auto x_ekf = ekf.predict(sys, u);
        auto x_ukf = ukf.predict(sys, u);
        
        // Orientation measurement
        {
            // We can measure the orientation every 5th step
            OrientationMeasurement orientation = om.h(x);
            
            // Measurement is affected by noise as well
            orientation.theta_r() += orientationNoise * noise(generator);
            orientation.theta_p() += orientationNoise * noise(generator);
            orientation.theta_y() += orientationNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(om, orientation);
            
            // Update UKF
            x_ukf = ukf.update(om, orientation);
        }
        
        // Position measurement
        {
            // We can measure the position every 10th step
            PositionMeasurement position = pm.h(x);
            
            // Measurement is affected by noise as well
            position.d1() += distanceNoise * noise(generator);
            position.d2() += distanceNoise * noise(generator);
            
            // Update EKF
            x_ekf = ekf.update(pm, position);
            
            // Update UKF
            x_ukf = ukf.update(pm, position);
        }
        
        // Print to stdout as csv format
        std::cout   << x.p_x() << "," << x.p_y() << "," << x.theta_r() << ","<< x.theta_p() << ","<< x.theta_y() << ","
                    << x_pred.p_x() << "," << x_pred.p_y() << "," << x_pred.theta_r() << ","<< x_pred.theta_p() << ","<< x_pred.theta_y() << ","
                    << x_ekf.p_x() << "," << x_ekf.p_y() << "," << x_ekf.theta_r() << ","<< x_ekf.theta_p() << ","<< x_ekf.theta_y() << ","
                    << x_ukf.p_x() << "," << x_ukf.p_y() << "," << x_ukf.theta_r() << ","<< x_ukf.theta_p() << ","<< x_ukf.theta_y()
                    << std::endl;
    }
    
    return 0;
}
