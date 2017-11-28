#pragma once

#include <math.h>
#include <vector>
#include <iostream>

using namespace std;

class Car {
public:
    int lane;
    double x;
    double y;
    double speed;
    double s;
    double d;
    double yaw;

    Car() {}

    Car(vector<double> &sensor_fusion) {
        double x = sensor_fusion[1];
        double y = sensor_fusion[2];
        double vx = sensor_fusion[3];
        double vy = sensor_fusion[4];
        // TODO s/d speed
        speed = sqrt(vx * vx + vy * vy);
        s = sensor_fusion[5];
        d = sensor_fusion[6];
        lane = d / 4;
    }

    void predict(int steps) {
        s += steps * .02 * speed;
    }

protected:
    double distanceTo(const Car &otherCar) {
        return otherCar.s - s;
    }

};


class EgoCar : public Car {
public:
    int target_lane;
    int lane_change_wp;
    double target_speed;
    double ref_x;
    double ref_y;
    double ref_yaw;

    EgoCar() {
        target_lane = 1;
    }

    void update(double x_, double y_, double speed_, double s_, double d_, double yaw_) {
        x = x_;
        y = y_;
        speed = speed_;
        s = s_;
        d = d_;
        yaw = yaw_;
        lane = d / 4;
        target_speed = 49.5;
        ref_x = x;
        ref_y = y;
        ref_yaw = deg2rad(yaw);
    }

    void plan(const vector<Car> &cars, vector<double> &previous_path_x, vector<double> &previous_path_y, vector<double> &maps_x, vector<double> &maps_y, double end_path_s) {
        int prev_size = previous_path_x.size();

        int next_wp;
        if (prev_size < 2) {
            next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, maps_x, maps_y);
        } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];
            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            next_wp = NextWaypoint(ref_x, ref_y, ref_yaw, maps_x, maps_y);

            s = end_path_s;
            speed = (sqrt((ref_x - ref_x_prev) * (ref_x - ref_x_prev) + (ref_y - ref_y_prev) * (ref_y - ref_y_prev)) / .02) * 2.237;
        }

        bool change_lanes = adjustRefVelIfShouldChangeLanes(cars);

        const int wp_delay = 2;
        if (change_lanes && ((next_wp - lane_change_wp) % maps_x.size() > wp_delay)) {
            double slowest = switchToLaneIfSafeZoneEmpty(cars, lane - 1, next_wp, 0);
            switchToLaneIfSafeZoneEmpty(cars, lane + 1, next_wp, slowest);
        }
    }

    double switchToLaneIfSafeZoneEmpty(const vector<Car> &cars, int checkLane, int next_wp, double slowest_so_far) {
        if (checkLane < 0 || checkLane > 2) return 0;

        double slowest = numeric_limits<double>::max();
        for (const Car &car : cars) {
            if (car.lane == checkLane) {
                double distance = distanceTo(car);
                bool notSafe = distance > -10 && distance < 20;
                if (notSafe) return 0;

                bool blocks = distance < 60 && car.speed < slowest;
                if (blocks) slowest = car.speed;
            }
        }
        if (slowest > slowest_so_far) {
            target_lane = checkLane;
            lane_change_wp = next_wp;
        }
        return slowest;
    }

private:

    bool adjustRefVelIfShouldChangeLanes(const vector<Car> &cars) {
        double closest = numeric_limits<double>::max();
        bool change_lanes = false;
        for (const Car &car : cars) {
            if (car.lane != lane) continue;

            double distance = distanceTo(car);
            if (distance > 0 && distance < 30 && distance < closest) {
                closest = distance;
                if (distance < 20) {
                    target_speed = car.speed * 2.237;
                }
                change_lanes = true;
            }
        }
        return change_lanes;
    }

};

