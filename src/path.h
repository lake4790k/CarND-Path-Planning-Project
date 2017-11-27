#include <vector>
#include "car.h"
#include "util.h"
#include "spline.h"

using namespace std;

class Path {
public:
    vector<double> next_x_vals;
    vector<double> next_y_vals;

    Path(EgoCar &ego, vector<double> &previous_path_x, vector<double> &previous_path_y,
         vector<double> &maps_s, vector<double> &maps_x, vector<double> &maps_y) {

        int prev_size = previous_path_x.size();
        vector<double> ptsx;
        vector<double> ptsy;

        if (prev_size < 2) {
            double prev_car_x = ego.x - cos(ego.yaw);
            double prev_car_y = ego.y - sin(ego.yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(ego.x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(ego.y);
        } else {
            ptsx.push_back(previous_path_x[prev_size - 2]);
            ptsx.push_back(previous_path_x[prev_size - 1]);

            ptsy.push_back(previous_path_y[prev_size - 2]);
            ptsy.push_back(previous_path_y[prev_size - 1]);
        }

        // TODO first offset based on closeness / speed?
        vector<double> next_wp0 = getXY(ego.s + 30, (2 + 4 * ego.target_lane), maps_s, maps_x, maps_y);
        vector<double> next_wp1 = getXY(ego.s + 60, (2 + 4 * ego.target_lane), maps_s, maps_x, maps_y);
        vector<double> next_wp2 = getXY(ego.s + 90, (2 + 4 * ego.target_lane), maps_s, maps_x, maps_y);

        ptsx.push_back(next_wp0[0]);
        ptsx.push_back(next_wp1[0]);
        ptsx.push_back(next_wp2[0]);

        ptsy.push_back(next_wp0[1]);
        ptsy.push_back(next_wp1[1]);
        ptsy.push_back(next_wp2[1]);

        for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ego.ref_x;
            double shift_y = ptsy[i] - ego.ref_y;

            ptsx[i] = (shift_x * cos(-ego.ref_yaw) - shift_y * sin(-ego.ref_yaw));
            ptsy[i] = (shift_x * sin(-ego.ref_yaw) + shift_y * cos(-ego.ref_yaw));
        }

        tk::spline s;
        s.set_points(ptsx, ptsy);

        for (int i = 0; i < previous_path_x.size(); i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
        }

        double target_x = 30.0;
        double target_y = s(target_x);
        double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));

        double x_add_on = 0;
        double car_speed = ego.speed;
        for (int i = 1; i <= 50 - previous_path_x.size(); i++) {

            if (ego.ref_vel > car_speed) {
                car_speed += .21;
            } else if (ego.ref_vel < car_speed) {
                car_speed -= .21;
            }

            double N = (target_dist / (.02 * car_speed / 2.24));
            double x_point = x_add_on + (target_x) / N;
            double y_point = s(x_point);

            x_add_on = x_point;

            double x_ref = x_point;
            double y_ref = y_point;

            x_point = (x_ref * cos(ego.ref_yaw) - y_ref * sin(ego.ref_yaw));
            y_point = (x_ref * sin(ego.ref_yaw) + y_ref * cos(ego.ref_yaw));

            x_point += ego.ref_x;
            y_point += ego.ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
        }
    }
};

