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

    static inline std::vector<int> split_i(std::string str, char delimiter) {
        std::vector<int> internal;
        std::stringstream ss(str);
        std::string temp;

        while (getline(ss, temp, delimiter)) {
            internal.push_back(std::atoi(temp.c_str()));
        }

        return internal;
    }

//    static inline double fRand_theta(double min, double max) {
//        double f = (double)rand() / RAND_MAX;
//        if(max > 2*M_PI){
//            return -M_PI + f * (2*M_PI);
//        }
//        return min + f * (max - min);
//    }

    template <typename T>
    std::vector<T> interpolation(T a, T b, T interval){
        std::vector<T> values;
        T tmp = a;
        while (tmp <= b){
            values.push_back(tmp);
            tmp = tmp + interval;
        }

        if(tmp > b)
            values.push_back(b);

        return values;
    }

    static inline bool random_bool() {
        return 0 + (rand() % (1 - 0 + 1)) == 1;
    }

    template <typename T>
    static inline T random_number(T min, T max){
        if(min > max)
            return 0;

        return min + T(rand() / (RAND_MAX / (max - min)));
    }
}

#endif