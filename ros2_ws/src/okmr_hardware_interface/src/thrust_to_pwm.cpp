// ros2 node class
// subscribe to /motor_thrust type: okmr_msgs/motor_thrust (see okmr_msgs for defintion) (default
// dur and reliability) subscribe to /voltage for voltage reference (transiet_local durability)
// /motor_throttle publisher type: okmr_msgs/motor_throttle
//
// ros2 parameter for thrust curve csv file (load on startup)
//
//
// in constructor, declare subs, publisher, and get the parameter (absolute path), then load the csv
// table (reference include/lazycsv.hpp)
//
// ex.
// std::vector<std::string_view> cities;
// for (const auto row : parser)
//{
//    const auto [city, state] = row.cells(0, 1);
//    cities.push_back(city.trimmed());
// }
//
// format in csv:
//
// voltage, throttle (pwm), thrust
// ex.  12, 1100, 2.5
//
// csv loading data structure:
// std::map<float (voltage), std::map<float (thrust), float(throttle)>>
//
// motor thrust callback:
//
//
// interpolation algorthim:
// round thrust value to 2 decimals
// check if thrust value already in the map
// if not, do linear search to find the index where requested thrust is < i, and > i-1
// then linearly interpolate between those 2 points
//
// similar to below, but with more descriptive variable and also using the above descibed data
// strucuture
for (size_t i = 0; i < g_thrusts.size () - 1; ++i) {
    double t1 = g_thrusts[i], t2 = g_thrusts[i + 1];
    if (thrust >= t1 && thrust <= t2) {
        double p1 = g_pwms[i], p2 = g_pwms[i + 1];
        double alpha = (thrust - t1) / (t2 - t1);
        return p1 + alpha * (p2 - p1);
    }
}
//
//
// on voltage callback, save to a voltage member variable
// on thrust callback, call the interpolation function on each array index with the cached voltage
// and the receieved thrust values
// then publish the interpolated array to /motor_throttle
