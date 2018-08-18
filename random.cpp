// Author: Akash Patel (apatel435@gatech.edu)

// Methods that deal with random operations

// Includes
#include <cstdlib>

#include "random.hpp"

// Namespaces
using namespace std;

// Functions
// // Random double value
double fRand(double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}
