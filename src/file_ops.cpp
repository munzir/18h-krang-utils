// Author: Akash Patel (apatel435@gatech.edu)

// Methods that deal with file operations

// Includes
#include "file_ops.hpp"

// Functions
// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, int stopCount,
                                      int lineToSkip, bool verbose) {
  // Read numbers (the pose params)
  ifstream infile;
  infile.open(inputPosesFilename);

  if (!infile.is_open()) {
    throw runtime_error(inputPosesFilename +
                        " can not be read, potentially does not exist!");
  }
  if (verbose)
    std::cout << "file " << inputPosesFilename << "successfully opened"
              << std::endl;

  int cols = 0, rows = 0;
  double buff[MAXBUFSIZE];

  int lineNumber = 1;
  while (!infile.eof() && rows <= stopCount) {
    if (lineNumber == lineToSkip) {
      string line;
      if (verbose) std::cout << "loading row " << rows << " ... ";
      getline(infile, line);
      if (verbose) std::cout << "done ";

      int temp_cols = 0;
      stringstream stream(line);
      if (verbose) std::cout << "col: " << std::endl;
      while (!stream.eof()) {
        stream >> buff[cols * rows + temp_cols++];
        if (verbose) std::cout << temp_cols << " ";
      }

      if (temp_cols == 0) {
        if (verbose) std::cout << "nothing";
        continue;
      }
      if (verbose) std::cout << std::endl;

      if (cols == 0) {
        cols = temp_cols;
        if (verbose) std::cout << "has " << cols << "columns" << std::endl;
      }
      rows++;
      lineNumber = 0;
    }
    lineNumber++;
  }

  if (verbose)
    std::cout << "closing file " << inputPosesFilename << " ... "
              << std::endl;
  infile.close();
  rows--;

  // cout << rows << endl;

  // Populate matrix with numbers.
  if (verbose) std::cout << "Populating matrix with numbers ... " << std::endl;
  Eigen::MatrixXd outputMatrix(rows, cols);
  for (int i = 0; i < rows; i++)
    for (int j = 0; j < cols; j++) outputMatrix(i, j) = buff[cols * i + j];


  if (verbose) std::cout << "done " << std::endl;
  return outputMatrix;
}

// // Read file as Matrix
Eigen::MatrixXd readInputFileAsMatrix(string inputPosesFilename, bool verbose) {
  return readInputFileAsMatrix(inputPosesFilename, INT_MAX, 1, verbose);
}

// M load_file (const std::string & path) {
//    std::ifstream indata;
//    indata.open(path);
//    std::string line;
//    std::vector<double> values;
//    uint rows = 0;
//    while (std::getline(indata, line)) {
//        std::stringstream lineStream(line);
//        std::string cell;
//        while (std::getline(lineStream, cell, ' ')) {
//            values.push_back(std::stod(cell));
//        }
//        ++rows;
//    }
//    return Map<const Matrix<typename M::Scalar, M::RowsAtCompileTime,
//    M::ColsAtCompileTime, RowMajor>>(values.data(), rows, values.size()/rows);
//}

// // Extract Filename
string extractFilename(string filename) {
  // Remove directory if present.
  // Do this before extension removal incase directory has a period character.
  const size_t last_slash_idx = filename.find_last_of("\\/");
  if (std::string::npos != last_slash_idx) {
    filename.erase(0, last_slash_idx + 1);
  }
  // Remove extension if present.
  const size_t period_idx = filename.rfind('.');
  if (std::string::npos != period_idx) {
    filename.erase(period_idx);
  }

  return filename;
}
