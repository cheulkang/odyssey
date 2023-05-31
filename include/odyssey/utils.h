//
// Created by mincheul on 23. 5. 16..
//

#ifndef ODYSSEY_UTILS_H
#define ODYSSEY_UTILS_H

#include <ros/ros.h>

namespace odyssey{
    // copied from geometry/angles/angles.h
    static inline double normalizeAnglePositive(double angle)
    {
        return fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
    }

    static inline double normalizeAngle(double angle)
    {
        double a = normalizeAnglePositive(angle);
        if (a > M_PI)
            a -= 2.0 * M_PI;
        return a;
    }

    static inline double shortestAngularDistance(double start, double end)
    {
        double res = normalizeAnglePositive(normalizeAnglePositive(end) - normalizeAnglePositive(start));
        if (res > M_PI)
        {
            res = -(2.0 * M_PI - res);
        }
        return normalizeAngle(res);
    }

    static inline std::vector<std::string> split(std::string str, char delimiter) {
        std::vector<std::string> internal;
        std::stringstream ss(str);
        std::string temp;

        while (getline(ss, temp, delimiter)) {
            internal.push_back(temp);
        }

        return internal;
    }

    static inline std::vector<double> split_d(std::string str, char delimiter) {
        std::vector<double> internal;
        std::stringstream ss(str);
        std::string temp;

        while (getline(ss, temp, delimiter)) {
            internal.push_back(std::atof(temp.c_str()));
        }

        return internal;
    }
    static inline std::vector<float> split_f(std::string str, char delimiter) {
        std::vector<float> internal;
        std::stringstream ss(str);
        std::string temp;

        while (getline(ss, temp, delimiter)) {
            internal.push_back(std::atof(temp.c_str()));
        }

        return internal;
    }

    double fRand_theta(double min, double max) {
        double f = (double)rand() / RAND_MAX;
        if(max > 2*M_PI){
            return -M_PI + f * (2*M_PI);
        }
        return min + f * (max - min);
    }
}

#endif