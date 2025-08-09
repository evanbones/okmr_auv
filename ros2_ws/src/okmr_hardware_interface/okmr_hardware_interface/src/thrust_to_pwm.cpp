/*
This is the interpolation function to get a PWM value from a desired thrust


*/


#include "thrust_to_pwm.h"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iostream>
#include <stdexcept>

static std::vector<double> g_pwms;
static std::vector<double> g_thrusts;
static int current_voltage_table = -1;

void loadThrustTable(double voltage) {
    std::string filename = "tables/" + std::to_string(voltage) + "V.csv";

    g_pwms.clear();
    g_thrusts.clear();

    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Cannot open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string pwm_str, thrust_str;
        if (std::getline(ss, pwm_str, ',') && std::getline(ss, thrust_str, ',')) {
            g_pwms.push_back(std::stod(pwm_str));
            g_thrusts.push_back(std::stod(thrust_str));
        }
    }

    std::cout << "[INFO] Loaded table: " << filename << "\n";
}

void updateThrustTableIfVoltageChanged(double voltage) {
    int rounded = std::round(voltage);
    if (rounded != current_voltage_table) {
        loadThrustTable(rounded);
        current_voltage_table = rounded;
    }
}

double interpolatePWM(double thrust) {
    if (g_thrusts.empty()) throw std::runtime_error("Thrust table is empty.");

    if (thrust <= g_thrusts.front()) return g_pwms.front();
    if (thrust >= g_thrusts.back()) return g_pwms.back();

    for (size_t i = 0; i < g_thrusts.size() - 1; ++i) {
        double t1 = g_thrusts[i], t2 = g_thrusts[i + 1];
        if (thrust >= t1 && thrust <= t2) {
            double p1 = g_pwms[i], p2 = g_pwms[i + 1];
            double alpha = (thrust - t1) / (t2 - t1);
            return p1 + alpha * (p2 - p1);
        }
    }

    return 1500; // Fallback
}

double thrustToPWM(double thrust) {
    return interpolatePWM(thrust);
}

double thrustToNormalizedPWM(double thrust) {
    return (interpolatePWM(thrust) - 1500.0) / 400.0;
}
