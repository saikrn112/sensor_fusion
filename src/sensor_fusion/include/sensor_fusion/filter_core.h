#ifndef Filter_CORE_H
#define FILTER_CORE_H

#include <armadillo>

namespace FilterCore {
  class EkfCore {
  private: //@TODO make sure that all the matrices are properly initialised while constructing the class
    arma::colvec state_;
    arma::mat processCovariance_;
    arma::mat estimateErrorCovariance_;
    arma::mat measurementNoiseCovariance_;
    arma::mat processNoiseCovariance_;
    arma::mat imuNoiseCovariance_;
    arma::mat disturbanceInfluenceMatrix_;
    arma::mat processMatrix_;
    arma::mat processMatrixJacobian_;

  public:
    EkfCore(); // all the inital parmeters will be updated as the equations are writtern
    void predict (const double delta);
    void update (const sensorMeasurements& measurement);
  }; // namespace FilterCore

} // namespace FilterCore

#endif
