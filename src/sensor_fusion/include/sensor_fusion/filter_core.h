#ifndef Filter_CORE_H
#define FILTER_CORE_H

namespace FilterCore {
  class EkfCore {
  private:
    
  public:
    EkfCore(); // all the inital parmeters will be updated as the equations are writtern
    void predict (const double delta);
    void update (const sensorMeasurements& measurement);
  }; // namespace FilterCore

} // namespace FilterCore

#endif
