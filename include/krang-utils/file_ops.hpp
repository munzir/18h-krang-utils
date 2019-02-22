// Author: Akash Patel (apatel435@gatech.edu)

// Methods that deal with file operations

// Includes
#include <dart/dart.hpp>
#include <vector>
#include <iostream>
#include <fstream>

// Namespaces
using namespace std;
using namespace Eigen;

// Defines
#define MAXBUFSIZE ((int) 1e6)

// Function Prototypes
// // Read file as matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, int stopCount, int lineToSkip);
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename);

//template<typename M>
//M load_file (const std::string & path);

// // Extract filename
string extractFilename(string filename);
