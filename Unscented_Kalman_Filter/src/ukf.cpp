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
    time_us = 0;

    is_initialized = false;

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
    n_x = 5;                          // state dimension (CTRV)
    x = VectorXd(n_x);                // state vector
    P = MatrixXd::Identity(n_x, n_x); // state covariance matrix
    P *= 0.25;

    /* Sigma points */
    n_aug = 7;
    n_sig = 2 * n_aug + 1; // number of sigma points
    Xsig_pred = MatrixXd(n_x, n_sig);
    lambda = 3 - n_aug;
    weights = VectorXd(n_sig);

    weights(0) = lambda / (lambda + n_aug);

    for (int i = 1; i < n_sig; i++)
    {
        weights(i) = 0.5 / (lambda + n_aug);
    }

    /* Process noise */
    std_a = 3;            // standard deviation longitudinal acceleration in m/s^2
    std_yawdd = M_PI;     // standard deviation yaw acceleration in rad/s^2
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package)
{
    long dt = meas_package.timestamp - time_us;
    time_us = meas_package.timestamp;

    if (is_initialized)
    {
        Prediction((double)dt / 1000000.0);
    }

    if (meas_package.sensor_type == meas_package.LASER)
    {
        if (!is_initialized)
        {
            x.fill(0.0);
            x(0) = meas_package.raw_measurements(0);
            x(1) = meas_package.raw_measurements(1);

            is_initialized = true;
        }
        else if (use_laser)
        {
            UpdateLidar(meas_package);
        }
    }
    else if (is_initialized && use_radar && meas_package.sensor_type == meas_package.RADAR)
    {
        UpdateRadar(meas_package);
    }
}

void UKF::Prediction(double dt)
{
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

    for (int i = 1; i <= n_aug; i++)
    {
        Xsig_aug.col(i)       = x_aug + sqrt(lambda + n_aug) * L.col(i-1);
        Xsig_aug.col(i+n_aug) = x_aug - sqrt(lambda + n_aug) * L.col(i-1);
    }

    /* Predict sigma points */
    for (int i = 0; i < n_sig; i++)
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

    for (int i = 0; i < n_sig; i++)
    {
        x = x + weights(i) * Xsig_pred.col(i);
    }

    for (int i = 0; i < n_sig; i++)
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
    /* Predict measurement */
    int n_z = 2;                     // measurement dimension
    MatrixXd H = MatrixXd(n_z, n_x); // measurement matrix

    H.fill(0.0);
    H(0, 0) = H(1, 1) = 1;

    VectorXd z_pred = H * x;

    /* Update state */
    VectorXd z = VectorXd(n_z); // measurement
    z(0) = meas_package.raw_measurements(0);
    z(1) = meas_package.raw_measurements(1);

    MatrixXd R = MatrixXd(n_z, n_z); // measurement covariance
    R.fill(0.0);
    R(0, 0) = std_laspx * std_laspx;
    R(1, 1) = std_laspy * std_laspy;

    VectorXd y = z - z_pred;
    MatrixXd S = H * P * H.transpose() + R;
    MatrixXd K = (P * H.transpose()) * S.inverse();
    MatrixXd I = MatrixXd::Identity(n_x, n_x);

    x = x + (K * y);
    P = (I - K * H) * P;

    /* Calculate lidar NIS */
    double epsilon = y.transpose() * S.inverse() * y;
}

void UKF::UpdateRadar(MeasurementPackage meas_package)
{
    /* Predict measurement */
    int n_z = 3;                          // measurement dimension
    MatrixXd Zsig = MatrixXd(n_z, n_sig); // measurement space sigma points
    VectorXd z_pred = VectorXd(n_z);      // mean predicted measurement
    MatrixXd S = MatrixXd(n_z, n_z);      // measurement covariance matrix

    for (int i = 0; i < n_sig; i++)
    {
        double px  = Xsig_pred(0, i);
        double py  = Xsig_pred(1, i);
        double v   = Xsig_pred(2, i);
        double yaw = Xsig_pred(3, i);

        Zsig(0, i) = sqrt(px * px + py * py); // r (range)
        Zsig(1, i) = atan2(py, px); // phi (azimuth)
        Zsig(2, i) = (px * cos(yaw) * v + py * sin(yaw) * v) / Zsig(0, i); // r_dot (range rate)
    }

    z_pred.fill(0.0);
    S.fill(0.0);

    for (int i = 0; i < n_sig; i++)
    {
        z_pred = z_pred + weights(i) * Zsig.col(i);
    }

    for (int i = 0; i < n_sig; i++)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;

        // angle normalization
        while (z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

        S = S + weights(i) * z_diff * z_diff.transpose();
    }

    S(0, 0) += std_radr * std_radr;
    S(1, 1) += std_radphi * std_radphi;
    S(2, 2) += std_radrd * std_radrd;

    /* Update state */
    MatrixXd Tc = MatrixXd(n_x, n_z); // cross correlation
    VectorXd z = VectorXd(n_z); // measurement

    z(0) = meas_package.raw_measurements(0);
    z(1) = meas_package.raw_measurements(1);
    z(2) = meas_package.raw_measurements(2);

    Tc.fill(0.0);

    for (int i = 0; i < n_sig; i++)
    {
        VectorXd z_diff = Zsig.col(i) - z_pred;
        VectorXd x_diff = Xsig_pred.col(i) - x;

        // angle normalization
        while (z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
        while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;
        while (x_diff(3) >  M_PI) x_diff(3) -= 2.0 * M_PI;
        while (x_diff(3) < -M_PI) x_diff(3) += 2.0 * M_PI;

        Tc = Tc + weights(i) * x_diff * z_diff.transpose();
    }

    MatrixXd K = Tc * S.inverse(); // kalman gain
    VectorXd z_diff = z - z_pred;

    // angle normalization
    while (z_diff(1) >  M_PI) z_diff(1) -= 2.0 * M_PI;
    while (z_diff(1) < -M_PI) z_diff(1) += 2.0 * M_PI;

    x = x + K * z_diff;
    P = P - K * S * K.transpose();

    /* Calculate radar NIS */
    double epsilon = z_diff.transpose() * S.inverse() * z_diff;
}
