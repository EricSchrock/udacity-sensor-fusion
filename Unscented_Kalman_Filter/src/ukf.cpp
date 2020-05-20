#include <iostream>

#include "ukf.h"
#include "Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF()
{
    use_laser = true;
    use_radar = true;

    /* Lidar noise values provided by the manufacturer (do not modify) */
    std_laspx = 0.15; // standard deviation position1 in m
    std_laspy = 0.15; // standard deviation position2 in m

    /* Radar noise values provided by the manufacturer (do not modify) */
    std_radr = 0.3;    // standard deviation radius in m
    std_radphi = 0.03; // standard deviation angle in rad
    std_radrd = 0.3;   // standard deviation radius change in m/s

    /* State */
    n_x = 5;                // state dimension (CTRV)
    x = VectorXd(n_x);      // state vector
    P = MatrixXd(n_x, n_x); // state covariance matrix

    /* Sigma points */
    n_aug = 7;
    n_sig = 2 * n_aug + 1; // number of sigma points
    Xsig_pred = MatrixXd(n_x, n_sig);
    lambda = 3 - n_aug;
    weights = VectorXd(n_sig);

    weights(0) = lambda / (lambda + n_aug);

    for (int i = 1; i < n_sig; ++i)
    {
        weights(i) = 0.5 / (lambda + n_aug);
    }

    /* Process noise */ //todo: set better values
    std_a = 30;     // standard deviation longitudinal acceleration in m/s^2
    std_yawdd = 30; // standard deviation yaw acceleration in rad/s^2

    is_initialized = false; //todo: use this (to init state vector?)
    time_us = 0;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    long dt = meas_package.timestamp - time_us;
    time_us = meas_package.timestamp;

    if (dt > 0)
    {
        Prediction((double)dt / 1000000.0);
    }

    if (use_radar && meas_package.sensor_type == meas_package.RADAR)
    {
        UpdateRadar(meas_package);
    }
    else if (use_laser && meas_package.sensor_type == meas_package.LASER)
    {
        UpdateLidar(meas_package);
    }
}

void UKF::Prediction(double dt)
{
    std::cout << "Prediction " << dt << std::endl;

    /* Generate sigma points */
    VectorXd x_aug = VectorXd(n_aug);
    MatrixXd P_aug = MatrixXd(n_aug, n_aug);
    MatrixXd Xsig_aug = MatrixXd(n_aug, n_sig);

    x_aug.head(n_x) = x;
    x_aug(5) = 0;
    x_aug(6) = 0;

    P_aug.fill(0.0);
    P_aug.topLeftCorner(n_x, n_x) = P;
    P_aug(5, 5) = std_a * std_a;
    P_aug(6, 6) = std_yawdd * std_yawdd;

    MatrixXd L = P_aug.llt().matrixL(); // square root of P

    Xsig_aug.col(0) = x_aug;

    for (int i = 1; i <= n_aug; ++i)
    {
        Xsig_aug.col(i)       = x_aug + sqrt(lambda + n_aug) * L.col(i-1);
        Xsig_aug.col(i+n_aug) = x_aug - sqrt(lambda + n_aug) * L.col(i-1);
    }

    /* Predict sigma points */
    for (int i = 0; i < n_sig; ++i)
    {
        double px       = Xsig_aug(0, i);
        double py       = Xsig_aug(1, i);
        double v        = Xsig_aug(2, i);
        double yaw      = Xsig_aug(3, i);
        double yawd     = Xsig_aug(4, i);
        double nu_a     = Xsig_aug(5, i);
        double nu_yawdd = Xsig_aug(6, i);

        if (fabs(yawd) > 0.001) // protect against divide by zero
        {
            px = px + (v / yawd) * (sin(yaw + yawd * dt) - sin(yaw));
            py = py + (v / yawd) * (cos(yaw) - cos(yaw + yawd * dt));
        }
        else
        {
            px = px + (v * dt * cos(yaw));
            py = py + (v * dt * sin(yaw));
        }

        yaw = yaw + (yawd * dt);

        // add noise
        Xsig_pred(0, i) = px + (0.5 * nu_a * dt * dt * cos(yaw));
        Xsig_pred(1, i) = py + (0.5 * nu_a * dt * dt * sin(yaw));
        Xsig_pred(2, i) = v + (nu_a * dt);
        Xsig_pred(3, i) = yaw + (0.5 * nu_yawdd * dt * dt);
        Xsig_pred(4, i) = yawd + (nu_yawdd * dt);
    }

    /* Predict mean and covariance */
    x.fill(0.0);
    P.fill(0.0);

    for (int i = 0; i < n_sig; ++i)
    {
        x = x + weights(i) * Xsig_pred.col(i);
    }

    for (int i = 0; i < n_sig; ++i)
    {
        VectorXd x_diff = Xsig_pred.col(i) - x;

        // angle normalization
        while (x_diff(3) >  M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

        P = P + weights(i) * x_diff * x_diff.transpose();
    }
}

void UKF::UpdateLidar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use lidar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the lidar NIS, if desired.
     */
    std::cout << "Lidar" << std::endl;

    /* Predict measurement */


    /* Update state */


    /* Calculate lidar NIS */

}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /**
     * TODO: Complete this function! Use radar data to update the belief
     * about the object's position. Modify the state vector, x_, and
     * covariance, P_.
     * You can also calculate the radar NIS, if desired.
     */
    std::cout << "Radar" << std::endl;

    /* Predict measurement */


    /* Update state */


    /* Calculate radar NIS */

}
