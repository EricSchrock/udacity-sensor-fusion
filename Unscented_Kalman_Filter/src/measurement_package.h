#ifndef MEASUREMENT_PACKAGE_H_
#define MEASUREMENT_PACKAGE_H_

#include "Eigen/Dense"

class MeasurementPackage
{
public:
    long timestamp;

    enum SensorType
    {
        LASER,
        RADAR
    } sensor_type;

    Eigen::VectorXd raw_measurements;
};

#endif /* MEASUREMENT_PACKAGE_H_ */