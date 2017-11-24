#include <fstream>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "util.h"
#include "car.h"
#include "path.h"

using namespace std;

using json = nlohmann::json;


int main() {
    uWS::Hub h;

    vector<double> maps_x;
    vector<double> maps_y;
    vector<double> maps_s;
    vector<double> maps_dx;
    vector<double> maps_dy;

    string map_file_ = "../data/highway_map.csv";
    double max_s = 6945.554;

    ifstream in_map_(map_file_.c_str(), ifstream::in);

    string line;
    while (getline(in_map_, line)) {
        istringstream iss(line);
        double x;
        double y;
        float s;
        float d_x;
        float d_y;
        iss >> x;
        iss >> y;
        iss >> s;
        iss >> d_x;
        iss >> d_y;
        maps_x.push_back(x);
        maps_y.push_back(y);
        maps_s.push_back(s);
        maps_dx.push_back(d_x);
        maps_dy.push_back(d_y);
    }

    EgoCar ego;

    h.onMessage([&ego, &maps_x, &maps_y, &maps_s, &maps_dx, &maps_dy](uWS::WebSocket<uWS::SERVER> ws, char *data,
                                                                                                                                     size_t length,
                                                                                                                                     uWS::OpCode opCode) {
        if (length && length > 2 && data[0] == '4' && data[1] == '2') {
            auto s = hasData(data);
            if (s != "") {
                auto j = json::parse(s);
                string event = j[0].get<string>();
                if (event == "telemetry") {

                    double car_x = j[1]["x"];
                    double car_y = j[1]["y"];
                    double car_s = j[1]["s"];
                    double car_d = j[1]["d"];
                    double car_yaw = j[1]["yaw"];
                    double car_speed = j[1]["speed"];
                    vector<double> previous_path_x = j[1]["previous_path_x"];
                    vector<double> previous_path_y = j[1]["previous_path_y"];
                    double end_path_s = j[1]["end_path_s"];
                    double end_path_d = j[1]["end_path_d"];
                    vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

                    int prev_size = previous_path_x.size();

                    ego.update(car_x, car_y, car_speed, car_s, car_d, car_yaw);

                    vector<Car> cars;
                    for (int i = 0; i < sensor_fusion.size(); i++) {
                        cars.emplace_back(sensor_fusion[i]);
                        cars[i].predict(prev_size);
                    }
                    ego.plan(cars, previous_path_x, previous_path_y, maps_x, maps_y, end_path_s);

                    Path path(ego, previous_path_x, previous_path_y, maps_s, maps_x, maps_y);

                    json msgJson;
                    msgJson["next_x"] = path.next_x_vals;
                    msgJson["next_y"] = path.next_y_vals;

                    auto msg = "42[\"control\"," + msgJson.dump() + "]";

                    //this_thread::sleep_for(chrono::milliseconds(1000));
                    ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

                }
            } else {
                // Manual driving
                std::string msg = "42[\"manual\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }
        }
    });

    // We don't need this since we're not using HTTP but if it's removed the
    // program
    // doesn't compile :-(
    h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                       size_t, size_t) {
        const std::string s = "<h1>Hello world!</h1>";
        if (req.getUrl().valueLength == 1) {
            res->end(s.data(), s.length());
        } else {
            // i guess this should be done more gracefully?
            res->end(nullptr, 0);
        }
    });

    h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
        std::cout << "Connected!!!" << std::endl;
    });

    h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                           char *message, size_t length) {
        ws.close();
        std::cout << "Disconnected" << std::endl;
    });

    int port = 4567;
    if (h.listen(port)) {
        std::cout << "Listening to port " << port << std::endl;
    } else {
        std::cerr << "Failed to listen to port" << std::endl;
        return -1;
    }
    h.run();
}
