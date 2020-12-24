#ifndef KALMAN_EXAMPLES1_UGV_SYSTEMMODEL_HPP_
#define KALMAN_EXAMPLES1_UGV_SYSTEMMODEL_HPP_

#include <kalman/LinearizedSystemModel.hpp>
#include <iostream>


namespace KalmanExamples
{
namespace UGV
{

/**
 * @brief System state vector-type for a complex 12DOF UGV
 *
 * This is a system state for a complex 12DOF UGV that
 * is characterized by its (p,v,theta,omega)-Position(3),Velocity(3),Orientation(3),Angular velocity(3).
 *
 * @param T Numeric scalar type
 */
template<typename T>
class State : public Kalman::Vector<T, 12>
{
public:
    KALMAN_VECTOR(State, T, 12)
    
    //! 0-11是12维向量的索引
    //! Position
    static constexpr size_t P_X = 0;
    static constexpr size_t P_Y = 1;
    static constexpr size_t P_Z = 2;
    //! Velocity
    static constexpr size_t V_X = 3;
    static constexpr size_t V_Y = 4;
    static constexpr size_t V_Z = 5;
    //! Orientation
    static constexpr size_t THETA_R = 6;
    static constexpr size_t THETA_P = 7;
    static constexpr size_t THETA_Y = 8;
    //! Angular velocity
    static constexpr size_t OMEGA_X = 9;
    static constexpr size_t OMEGA_Y = 10;
    static constexpr size_t OMEGA_Z = 11;
    
    T p_x()         const { return (*this)[ P_X ]; }
    T p_y()         const { return (*this)[ P_Y ]; }
    T p_z()         const { return (*this)[ P_Z ]; }
    T v_x()         const { return (*this)[ V_X ]; }
    T v_y()         const { return (*this)[ V_Y ]; }
    T v_z()         const { return (*this)[ V_Z ]; }
    T theta_r()     const { return (*this)[ THETA_R ]; }
    T theta_p()     const { return (*this)[ THETA_P ]; }
    T theta_y()     const { return (*this)[ THETA_Y ]; }
    T omega_x()     const { return (*this)[ OMEGA_X ]; }
    T omega_y()     const { return (*this)[ OMEGA_Y ]; }
    T omega_z()     const { return (*this)[ OMEGA_Z ]; }
    
    T& p_x()         { return (*this)[ P_X ]; }
    T& p_y()         { return (*this)[ P_Y ]; }
    T& p_z()         { return (*this)[ P_Z ]; }
    T& v_x()         { return (*this)[ V_X ]; }
    T& v_y()         { return (*this)[ V_Y ]; }
    T& v_z()         { return (*this)[ V_Z ]; }
    T& theta_r()     { return (*this)[ THETA_R ]; }
    T& theta_p()     { return (*this)[ THETA_P ]; }
    T& theta_y()     { return (*this)[ THETA_Y ]; }
    T& omega_x()     { return (*this)[ OMEGA_X ]; }
    T& omega_y()     { return (*this)[ OMEGA_Y ]; }
    T& omega_z()     { return (*this)[ OMEGA_Z ]; }
};

/**
 * @brief System control-input vector-type for a complex 12DOF UGV
 *
 * This is the system control-input of a complex 12DOF UGV that
 * can control the velocity in its current direction as well as the
 * change in direction.
 *
 * @param T Numeric scalar type
 */
template<typename T>
class Control : public Kalman::Vector<T, 6>
{
public:
    KALMAN_VECTOR(Control, T, 6)
    
    //! Acceleration
    static constexpr size_t A_X = 0;
    static constexpr size_t A_Y = 1;
    static constexpr size_t A_Z = 2;
    //! Angular velocity
    static constexpr size_t OMEGA_X = 3;
    static constexpr size_t OMEGA_Y = 4;
    static constexpr size_t OMEGA_Z = 5;
    
    T a_x()       const { return (*this)[ A_X ]; }
    T a_y()       const { return (*this)[ A_Y ]; }
    T a_z()       const { return (*this)[ A_Z ]; }
    T omega_x()   const { return (*this)[ OMEGA_X ]; }
    T omega_y()   const { return (*this)[ OMEGA_Y ]; }
    T omega_z()   const { return (*this)[ OMEGA_Z ]; }
    
    T& a_x()        { return (*this)[ A_X ]; }
    T& a_y()        { return (*this)[ A_Y ]; }
    T& a_z()        { return (*this)[ A_Z ]; }
    T& omega_x()    { return (*this)[ OMEGA_X ]; }
    T& omega_y()    { return (*this)[ OMEGA_Y ]; }
    T& omega_z()    { return (*this)[ OMEGA_Z ]; }
};

/**
 * @brief System model for a complex 12DOF UGV
 *
 * This is the system model defining how UGV moves from one 
 * time-step to the next, i.e. how the system state evolves over time.
 *
 * @param T Numeric scalar type
 * @param CovarianceBase Class template to determine the covariance representation
 *                       (as covariance matrix (StandardBase) or as lower-triangular
 *                       coveriace square root (SquareRootBase))
 */
template<typename T, template<class> class CovarianceBase = Kalman::StandardBase>
class SystemModel : public Kalman::LinearizedSystemModel<State<T>, Control<T>, CovarianceBase>
{
public:
    //! State type shortcut definition
	typedef KalmanExamples::UGV::State<T> S;
    
    //! Control type shortcut definition
    typedef KalmanExamples::UGV::Control<T> C;
    
    /**
     * @brief Definition of (non-linear) state transition function
     *
     * This function defines how the system state is propagated through time,
     * i.e. it defines in which state \f$\hat{x}_{k+1}\f$ is system is expected to 
     * be in time-step \f$k+1\f$ given the current state \f$x_k\f$ in step \f$k\f$ and
     * the system control input \f$u\f$.
     *
     * @param [in] x The system state in current time-step
     * @param [in] u The control vector input
     * @returns The (predicted) system state in the next time-step
     */
    S f(const S& x, const C& u) const
    {
        //! Predicted state vector after transition
        S x_;
        
        // New postion given by old postion plus postion change
        auto newXPosition = x.p_x() + x.v_x() + 0.5 * u.a_x();
        auto newYPosition = x.p_y() + x.v_y() + 0.5 * u.a_y();
        auto newZPosition = x.p_z() + x.v_z() + 0.5 * u.a_z();
        // New velocity given by old velocity plus velocity change
        auto newXVelocity = x.v_x() + u.a_x();
        auto newYVelocity = x.v_y() + u.a_y();
        auto newZVelocity = x.v_z() + u.a_z();
        // New orientation given by old orientation plus orientation change
        auto newROrientation = x.theta_r() + u.omega_x();
        auto newPOrientation = x.theta_p() + u.omega_y();
        auto newYOrientation = x.theta_y() + u.omega_z();

        // Predict state vector
        x_.p_x() = newXPosition;        
        x_.p_y() = newYPosition;        
        x_.p_z() = newZPosition;
        x_.v_x() = newXVelocity;
        x_.v_y() = newYVelocity;
        x_.v_z() = newZVelocity;
        x_.theta_r() = newROrientation;
        x_.theta_p() = newPOrientation;
        x_.theta_y() = newYOrientation;
        x_.omega_x() = u.omega_x();
        x_.omega_y() = u.omega_y();
        x_.omega_z() = u.omega_z();
        
        // Return transitioned state vector
        return x_;
    }
    
protected:
    /**
     * @brief Update jacobian matrices for the system state transition function using current state
     *
     * This will re-compute the (state-dependent) elements of the jacobian matrices
     * to linearize the non-linear state transition function \f$f(x,u)\f$ around the
     * current state \f$x\f$.
     *
     * @note This is only needed when implementing a LinearizedSystemModel,
     *       for usage with an ExtendedKalmanFilter or SquareRootExtendedKalmanFilter.
     *       When using a fully non-linear filter such as the UnscentedKalmanFilter
     *       or its square-root form then this is not needed.
     *
     * @param x The current system state around which to linearize
     * @param u The current system control input
     */
    // void updateJacobians( const S& x, const C& u )
    // {
    //     // F = df/dx (Jacobian of state transition w.r.t. the state)
    //     this->F.setZero();
        
    //     // partial derivative of x.x() w.r.t. x.x()
    //     this->F( S::X, S::X ) = 1;
    //     // partial derivative of x.x() w.r.t. x.theta()
    //     this->F( S::X, S::THETA ) = -std::sin( x.theta() + u.dtheta() ) * u.v();
        
    //     // partial derivative of x.y() w.r.t. x.y()
    //     this->F( S::Y, S::Y ) = 1;
    //     // partial derivative of x.y() w.r.t. x.theta()
    //     this->F( S::Y, S::THETA ) = std::cos( x.theta() + u.dtheta() ) * u.v();
        
    //     // partial derivative of x.theta() w.r.t. x.theta()
    //     this->F( S::THETA, S::THETA ) = 1;
        
    //     // W = df/dw (Jacobian of state transition w.r.t. the noise)
    //     this->W.setIdentity();
    //     // TODO: more sophisticated noise modelling
    //     //       i.e. The noise affects the the direction in which we move as 
    //     //       well as the velocity (i.e. the distance we move)
    // }
};

} // namespace UGV
} // namespace KalmanExamples

#endif