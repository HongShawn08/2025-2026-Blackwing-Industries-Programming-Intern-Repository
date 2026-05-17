/*
#pragma once
#include <vector>
#include <cmath>
#include <limits>
#include <stdexcept>

struct StreamingStats {
private:
    long long n;    // Count of processed elements
    double mean;    // Current running mean
    double M2;      // Sum of squares of differences from the current mean

public:
    // Constructor
    StreamingStats() : n(0), mean(0.0), M2(0.0) {}

    // Resets the accumulator
    void clear() {
        n = 0;
        mean = 0.0;
        M2 = 0.0;
    }

    // Update the statistics with a new value x
    void push(double x) {
        n++;
        double delta = x - mean;
        mean += delta / n;
        double delta2 = x - mean;
        M2 += delta * delta2;
    }

    // Get the number of samples
    long long count() const {
        return n;
    }

    // Get the current Mean
    double getMean() const {
        if (n == 0) return 0.0; // Or return NaN depending on preference
        return mean;
    }

    // Get Sample Variance (divide by n - 1)
    // Returns 0 if fewer than 2 samples
    double getVariance() const {
        if (n < 2) return 0.0;
        return M2 / (n - 1);
    }

    // Get Population Variance (divide by n)
    double getPopulationVariance() const {
        if (n == 0) return 0.0;
        return M2 / n;
    }

    // Get Sample Standard Deviation
    double getStdDev() const {
        return std::sqrt(getVariance());
    }
};
*/