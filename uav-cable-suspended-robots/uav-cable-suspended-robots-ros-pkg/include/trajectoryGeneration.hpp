#include <vector>
#include <cmath>

void cubicVelTraj(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, double, double, double, double, double, double);
void trapVelTraj(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, double, double, double, double, double);
void trapVelTraj_tf(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, double, double, double, double, double);
void sinusoidalTraj(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, double, double, double, double, double);
void dampedSinusoidalTraj(std::vector<double>&, std::vector<double>&, std::vector<double>&, std::vector<double>&, double, double, double, double, double, double);


// Polynomial cubic trajectory function
// void cubicVelTraj(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sDot, std::vector<double>& sDotDot, 
//     double dt, double tFinal, double sInit, double sFinal, double sDotInit, double sDotFinal);

// Definition of the multiJointTrapVelTraj function
void multiJointCubicVelTraj(double dt, const std::vector<double>& tFinal, 
                           const std::vector<double>& sInit, const std::vector<double>& sFinal, 
                           const std::vector<double>& sDotInit, const std::vector<double>& sDotFinal,
                           std::vector<std::vector<double>>& s, std::vector<std::vector<double>>& sdot, 
                           std::vector<std::vector<double>>& sdotdot, std::vector<std::vector<double>>& t);