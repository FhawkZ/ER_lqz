// Minimal error recovery utility for Franka FCI
#include <franka/exception.h>
#include <franka/robot.h>
#include <iostream>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  try {
    franka::Robot robot(argv[1]);
    std::cout << "Running automatic error recovery on " << argv[1] << "..." << std::endl;
    robot.automaticErrorRecovery();
    std::cout << "Error recovery finished successfully." << std::endl;
  } catch (const franka::Exception& e) {
    std::cerr << e.what() << std::endl;
    return -1;
  }
  return 0;
}
