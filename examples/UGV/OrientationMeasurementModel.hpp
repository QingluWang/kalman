#ifndef KALMAN_EXAMPLES_UGV_ORIENTATIONMEASUREMENTMODEL_HPP_
#define KALMAN_EXAMPLES_UGV_ORIENTATIONMEASUREMENTMODEL_HPP_

#include <kalman/LinearizedMeasurementModel.hpp>

namespace KalmanExamples
{
namespace UGV
{

/**
 * @brief Measurement vector measuring an orientation (i.e. by using a 9DOF IMU)
 *
 * @param T Numeric scalar type
 */
template<typename T>
class OrientationMeasurement : public Kalman::Vector<T, 3>
{
public:
    KALMAN_VECTOR(OrientationMeasurement, T, 3)
    
    //! Orientation
    static constexpr size_t THETA_R = 0;
    static constexpr size_t THETA_P = 0;
    static constexpr size_t THETA_Y = 0;
        
    T theta_r()     const { return (*this)[ THETA_R ]; }
    T theta_p()     const { return (*this)[ THETA_P ]; }
    T theta_y()     const { return (*this)[ THETA_Y ]; }
    T& theta_r()          { return (*this)[ THETA_R ]; }
    T& theta_p()          { return (*this)[ THETA_P ]; }
    T& theta_y()          { return (*this)[ THETA_Y ]; }
};

/**
 * @brief Measurement model for measuring orientation of a 12DOF UGV
 *
 * This is the measurement model for measuring the orientation of our
 * UGV. This could be realized by an IMU.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class OrientationMeasurementModel : public Kalman::LinearizedMeasurementModel<State<T>, OrientationMeasurement<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
    typedef KalmanExamples::UGV::State<T> S;
    
    //! Measurement type shortcut definition
    typedef  KalmanExamples::UGV::OrientationMeasurement<T> M;
    
    OrientationMeasurementModel()
    {
        // Setup jacobians. As these are static, we can define them once
        // and do not need to update them dynamically
        this->H.setIdentity();
        this->V.setIdentity();
    }
    
    /**
     * @brief Definition of (possibly non-linear) measurement function
     *
     * This function maps the system state to the measurement that is expected
     * to be received from the sensor assuming the system is currently in the
     * estimated state.
     *
     * @param [in] x The system state in current time-step
     * @returns The (predicted) sensor measurement for the system state
     */
    M h(const S& x) const
    {
        M measurement;
        
        // Measurement is given by the actual robot orientation
        measurement.theta_r() = x.theta_r();
        measurement.theta_p() = x.theta_p();
        measurement.theta_y() = x.theta_y();
        
        return measurement;
    }
};

} // namespace UGV
} // namespace KalmanExamples

#endif