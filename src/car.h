#pragma once

#include <math.h>
#include <vector>

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

    double distance(Car &otherCar) {
        return otherCar.s - s;
    }

};


class EgoCar : public Car {

public:
    int target_lane;
    double ref_vel;
    double ref_x;
    double ref_y;
    double ref_yaw;

    int next_wp;
    int lane_change_wp;

    EgoCar() {}

    void update(double x_, double y_, double speed_, double s_, double d_, double yaw_) {
        x = x_;
        y = y_;
        speed = speed_;
        s = s_;
        d = d_;
        lane = d / 4;
        yaw = yaw_;

        next_wp = -1;
        ref_vel = 49.5;

        ref_x = x;
        ref_y = y;
        ref_yaw = deg2rad(yaw);
    }

    void plan(vector<Car> &cars, vector<double> &previous_path_x, vector<double> &previous_path_y, vector<double> &maps_x, vector<double> &maps_y, double end_path_s) {
        int prev_size = previous_path_x.size();

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

        double closestDist = 100000;
        bool change_lanes = false;
        for (Car &car : cars) {
            if (car.lane != lane) continue;

            double dist = distance(car);
            if (dist > 0 && dist < 30 && dist < closestDist) {
                closestDist = dist;
                if (dist < 20) {
                    ref_vel = car.speed * 2.237;
                } else {
                    // TODO only if can't change lanes?
                    //go slightly slower than the cars speed
                    ref_vel = car.speed * 2.237 - 5;
                }
                change_lanes = true;
            }
        }

        target_lane = lane;
        if (change_lanes && ((next_wp - lane_change_wp) % maps_x.size() > 2)) {
            bool changed_lanes = false;
            if (lane != 0 && !changed_lanes) {
                bool lane_safe = true;

                for (Car &car : cars) {
                    if (car.lane == lane - 1) {
                        double dist = distance(car);
                        if (dist < 20 && dist > -20) {
                            lane_safe = false;
                        }
                    }
                }
                if (lane_safe) {
                    changed_lanes = true;
                    lane -= 1;
                    lane_change_wp = next_wp;
                }
            }
            if (lane != 2 && !changed_lanes) {
                bool lane_safe = true;
                for (Car &car : cars) {
                    if (car.lane == lane + 1) {
                        double dist = distance(car);
                        if (dist < 20 && dist > -20) {
                            lane_safe = false;
                        }
                    }
                }
                if (lane_safe) {
                    changed_lanes = true;
                    lane += 1;
                    lane_change_wp = next_wp;
                }
            }
        }
    }
};

