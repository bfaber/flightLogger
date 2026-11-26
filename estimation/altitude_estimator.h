
#pragma once

#include <cmath>

class AltitudeEstimator {
public:
    void setReference(double pressure_pa) {
        reference_pa_ = pressure_pa;
    }

    bool hasReference() const {
        return reference_pa_ > 0.0;
    }

    double estimate(double pressure_pa) const {
        if (!hasReference() || pressure_pa <= 0.0)
            return 0.0;

        return 44330.0 *
            (1.0 - std::pow(
                pressure_pa / reference_pa_,
                0.19029496));
    }

private:
    double reference_pa_ = 0.0;
};
