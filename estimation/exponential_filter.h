
#pragma once

class ExponentialFilter {
public:
    explicit ExponentialFilter(double alpha)
        : alpha_(alpha) {}

    double update(double measurement) {
        if (!initialized_) {
            value_ = measurement;
            initialized_ = true;
        } else {
            value_ += alpha_ * (measurement - value_);
        }

        return value_;
    }

    void reset() {
        initialized_ = false;
        value_ = 0.0;
    }

    double value() const {
        return value_;
    }

private:
    double alpha_;
    double value_ = 0.0;
    bool initialized_ = false;
};
