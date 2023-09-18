#include "trajectoryGeneration.hpp"

// Polynomial cubic trajectory function
void cubicVelTraj(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sDot, std::vector<double>& sDotDot, 
    double dt, double tFinal, double sInit, double sFinal, double sDotInit, double sDotFinal) {

    double a0, a1, a2, a3;                                                      // Coefficient of the polynomial
    int nSamples, initialSize;
    nSamples = (int)(tFinal/dt);                                                // Compute number of samples

    a0 = sInit;                                                                 // Compute terms of the polynomial
    a1 = sDotInit;
    a3 = (sDotFinal*tFinal - 2*sFinal + a1*tFinal + 2*a0)/pow(tFinal,3);
    a2 = (sFinal - a3*pow(tFinal,3) - a1*tFinal - a0)/pow(tFinal,2);

    initialSize = t.size();
    if (initialSize == 0) {
        t.push_back(0);                                                             // First time value
    } else{
        t.push_back(t[initialSize-1]+dt);                                           // First time value
    }

    for (int k=0; k<=nSamples; k++) {                                           // For-loop for the generation of the trajectory
        if (k!=0) t.push_back(t[initialSize-1] + t[k-1] + dt);                  // Time vector
        s.push_back(a3*pow(t[k],3) + a2*pow(t[k],2) + a1*t[k] + a0);            // Zero-derivative
        sDot.push_back(3*a3*pow(t[k],2) + 2*a2*t[k] + a1);                      // First derivative
        sDotDot.push_back(6*a3*t[k] + 2*a2);                                    // Second derivative
    }
}


// Definition of the multiJointTrapVelTraj function
void multiJointCubicVelTraj(double dt, const std::vector<double>& tFinal, 
                           const std::vector<double>& sInit, const std::vector<double>& sFinal, 
                           const std::vector<double>& sDotInit, const std::vector<double>& sDotFinal,
                           std::vector<std::vector<double>>& s, std::vector<std::vector<double>>& sdot, 
                           std::vector<std::vector<double>>& sdotdot, std::vector<std::vector<double>>& t) {

    int numJoints = sInit.size();

    s.resize(numJoints);
    sdot.resize(numJoints);
    sdotdot.resize(numJoints);
    t.resize(numJoints);

    for (int i = 0; i < numJoints; i++) {
        double tFinal_i = tFinal[i];
        double sInit_i = sInit[i];
        double sFinal_i = sFinal[i];
        double sDotInit_i = sDotInit[i];
        double sDotFinal_i = sDotFinal[i];

        std::vector<double> s_i, sdot_i, sdotdot_i, t_i;
        cubicVelTraj(t[i], s[i], sdot[i] , sdotdot[i], dt, tFinal_i, sInit_i, sFinal_i, sDotInit_i, sDotFinal_i);
    }
}



// Step function with y = 0 at t = 0
double u(double t) {
    return (t >= 0) ? 1 : 0;
}

// Step function with y = 1 at t = 0
double u0(double t) {
    return (t > 0) ? 1 : 0;
}


// This function returns the timing law of the curvilinear abscissa according
// to the input parameters.
// The curvilinear abscissa has the following characteristics
//   - s : parabolic linear trend
//   - sdot : trapezoidal profile
//   - sdotdot : piecewise function
// INPUT PARAMETERS
//   - dt : sampling time
//   - acc : desired cruise acceleration
//   - velDes : desired cruise velocity
//   - sInit : initial value of curvilinear abscissa
//   - sFinal : final value of curvilinear abscissa
// OUTPUT PARAMETERS
//   - s : curvilinear abscissa trend
//   - sdot : curvilinear abscissa velocity
//   - sdotdot : curvilinear abscissa acceleration
//   - t : trajectory time interval

void trapVelTraj(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sdot, std::vector<double>& sdotdot,
    double dt, double accDes, double velDes, double sInit, double sFinal) {

    // Check on the signum of velocity and acceleration
    if (sFinal-sInit<0) {
        velDes = -velDes;
        accDes = -accDes;
    }

    double velPerc = 1.0;

    // Find tFinal
    double cruiseVel = velPerc * velDes;
    double tAcc = abs(cruiseVel / accDes);
    double tFinal = (accDes * pow(tAcc, 2) + sFinal - sInit) / (accDes * tAcc);

    // Check on the trajectory feasibility
    while (abs(cruiseVel) >= 2 * abs(sFinal - sInit) / tFinal || abs(cruiseVel) <= abs(sFinal - sInit) / tFinal) {
        velPerc -= 0.05;
        cruiseVel = velPerc * velDes;
        tAcc = abs(cruiseVel / accDes);
        tFinal = (accDes * pow(tAcc, 2) + sFinal - sInit) / (accDes * tAcc);
    }

    // Calculate s, sdot, sdotdot, t
    int n = static_cast<int>(round(tFinal / dt)) + 1;   
    t.resize(n);
    s.resize(n);
    sdot.resize(n);
    sdotdot.resize(n);

    // Curvilinear Abscissa generator
    for (int k = 0; k < n; k++) {
        double time = k * dt;
        t[k] = time;
        
        s[k] = (sInit + 0.5 * accDes * pow(time, 2)) * u(time) * u(tAcc - time) +
            (sInit + cruiseVel * (time - tAcc / 2)) * u0(time - tAcc) * u(tFinal - tAcc - time) +
            (sFinal - 0.5 * accDes * pow(tFinal - time, 2)) * u0(time - (tFinal - tAcc)) * u(tFinal - time) +
            sFinal * u0(time - tFinal);

        sdot[k] = accDes * time * u(time) * u(tAcc - time) +
            cruiseVel * u0(time - tAcc) * u(tFinal - tAcc - time) +
            (cruiseVel - accDes * (time - (tFinal - tAcc))) * u0(time - (tFinal - tAcc)) * u(tFinal - time) + 
            0 * u0(time - tFinal);

        sdotdot[k] = accDes * u(time) * u(tAcc - time) +
            0 *u0(time - tAcc) * u(tFinal - tAcc - time) + 
            -accDes * u0(time - (tFinal - tAcc)) * u(tFinal - time) +
            0 * u0(time - tFinal);
    }
}



// This function returns the timing law of the curvilinear abscissa according
// to the input parameters. 
// The curvilinear abscissa has the following characteristics
//                           - s : parabolic linear trend
//                           - sdot : trapezoidal profile
//                           - sdotdot : piecewise function
// INPUT PARAMETERS
//                           - dt : sampling time
//                           - acc : desired cruise acceleration
//                           - tFinal : trajectory duration
//                           - sInit : initial value of curvilinear abscissa
//                           - sFinal : final value of curvilinear abscissa
// OUTPUT PARAMETERS
//                           - s : curvilinear abscissa trend
//                           - sdot : curvilinear abscissa velocity
//                           - sdotdot : curvilinear abscissa acceleration
//                           - t : trajectory time interval

void trapVelTraj_tf(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sdot, std::vector<double>& sdotdot,
    double dt, double tFinal, double accDes, double sInit, double sFinal) {

    // Find feasible acceleration
    double sqrtArg = (tFinal * tFinal * accDes - 4 * abs(sFinal - sInit)) / accDes;

    double acc, tAcc;
    if (sqrtArg < 0) {
        acc = 4 * abs(sFinal - sInit) / (tFinal * tFinal);
        tAcc = tFinal / 2;
    }
    else {
        acc = accDes;
        tAcc = tFinal / 2 - 0.5 * sqrt(sqrtArg);
    }

    // Check on the signum of the acceleration
    if ((sFinal-sInit)<0) {
        acc = -acc;
    }

    // Find cruiseVel
    double cruiseVel = tAcc * acc;

    // Calculate s, sdot, sdotdot, t
    int n = static_cast<int>(round(tFinal / dt)) + 1;
    t.resize(n);
    s.resize(n);
    sdot.resize(n);
    sdotdot.resize(n);

    // Curvilinear Abscissa generator
    for (int k = 0; k < n; k++) {
        double time = k * dt;
        t[k] = time;

        s[k] = (sInit + 0.5 * acc * pow(time, 2)) * u(time) * u(tAcc - time) +
            (sInit + cruiseVel * (time - tAcc / 2)) * u0(time - tAcc) * u(tFinal - tAcc - time) +
            (sFinal - 0.5 * acc * pow(tFinal - time, 2)) * u0(time - (tFinal - tAcc)) * u(tFinal - time) +
            sFinal * u0(time - tFinal);

        sdot[k] = acc * time * u(time) * u(tAcc - time) +
            cruiseVel * u0(time - tAcc) * u(tFinal - tAcc - time) +
            (cruiseVel - acc * (time - (tFinal - tAcc))) * u0(time - (tFinal - tAcc)) * u(tFinal - time) + 
            0 * u0(time - tFinal);

        sdotdot[k] = acc * u(time) * u(tAcc - time) +
            0 *u0(time - tAcc) * u(tFinal - tAcc - time) + 
            -acc * u0(time - (tFinal - tAcc)) * u(tFinal - time) +
            0 * u0(time - tFinal);
    }
}



void sinusoidalTraj(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sdot, std::vector<double>& sdotdot,
    double dt, double tFinal, double amp, double omega, double phase) {

    // Calculate s, sdot, sdotdot, t
    int n = static_cast<int>(round(tFinal / dt)) + 1;
    t.resize(n);
    s.resize(n);
    sdot.resize(n);
    sdotdot.resize(n);

    for (int k = 0; k < n; k++) {
        double time = k * dt;
        t[k] = time;
        
        s[k] = amp*sin(omega*time-phase);

        sdot[k] = amp*omega*cos(omega*time-phase);

        sdotdot[k] = -amp*pow(omega,2)*sin(omega*time-phase);
    }
}


void dampedSinusoidalTraj(std::vector<double>& t, std::vector<double>& s, std::vector<double>& sdot, std::vector<double>& sdotdot,
    double dt, double tFinal, double amp, double omega, double phase, double decay_rate) {

    // Calculate s, sdot, sdotdot, t
    int n = static_cast<int>(round(tFinal / dt)) + 1;
    t.resize(n);
    s.resize(n);
    sdot.resize(n);
    sdotdot.resize(n);

    for (int k = 0; k < n; k++) {
        double time = k * dt;
        t[k] = time;

        s[k] = amp * exp(-decay_rate * time) * cos(omega * time + phase);

        sdot[k] = -amp * exp(-decay_rate * time) * (decay_rate * cos(phase + omega * time) + omega * sin(phase + omega * time));

        sdotdot[k] = amp * exp(-decay_rate * time) * (pow(decay_rate,2) * cos(phase + omega * time) - pow(omega,2)  * cos(phase + omega * time) + 2 * decay_rate * omega * sin(phase + omega * time));
    }
}


