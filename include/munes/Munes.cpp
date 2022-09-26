#include "Munes.h"

double calculateAltitude(const double pressure, const double criteria_pressure)
{
    return 44330*(1-std::pow(pressure/criteria_pressure,1/5.255));
}

double getRandomInRange(double min, double max) {
    static const double fraction = 1.0 / (RAND_MAX + 1.0);
    return min + (max - min) * (std::rand() * fraction);
}