#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include <math.h>

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

using namespace std;

// for convenience
using json = nlohmann::json;


bool isLaneChangePossible(vector<vector<double>> sensor_fusion, double car_x, double car_y,  double car_s,  double car_d, double car_yaw, double car_speed, int& car_lane, int prev_size,
		vector<double> previous_path_x, vector<double> previous_path_y, vector<double> map_waypoints_s, vector<double> map_waypoints_x, vector<double> map_waypoints_y, double ref_vel) {
	// go through the sensor fusion data
	vector<bool> laneSafe = {true, true, true};

	std::cout << "NUMBER_VEHICLES: " << sensor_fusion.size() << std::endl;
	if(sensor_fusion.size() > 0) {
		for(int i=0; i < sensor_fusion.size(); ++i) {
			float d = sensor_fusion[i][6];
			int current_lane_check = (int)(d / 4);
			std::cout << "VECHICLE d: " << d << "Lane: " << current_lane_check << std::endl;

			// car not in my lane 
			if(current_lane_check != car_lane) {
				double vx = sensor_fusion[i][3];
				double vy = sensor_fusion[i][4];
				double check_speed = sqrt(vx*vx + vy *vy);
				double check_car_s = sensor_fusion[i][5];

				// Using speed we can predict where the car will be in the future

				// If using previous points, we can project s value out
				check_car_s += ((double) prev_size * 0.02 * check_speed);
        // our car s
				double predicted_car_s =  car_s + ((double) prev_size * 0.02 * check_speed);

				if(abs(check_car_s - predicted_car_s) < 50) {
					std::cout << "COLLISSION IN LANE:" << current_lane_check << std::endl;
					laneSafe[current_lane_check] = false;
				}
			}
		}
	}

	for(int i=0; i < laneSafe.size(); ++i) {
		if(laneSafe[i] && i != car_lane && abs(i-car_lane) < 2) {
			car_lane = i;
			break;
		}
	}
}

int main() {
	uWS::Hub h;


/*
Each waypoint in the list contains [x,y,s,dx,dy] values. 
x and y are the waypoint's map coordinate position, 
the s value is the distance along the road to get to 
that waypoint in meters, the dx and dy values define 
the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, 
distance along the road, goes from 0 to 6945.554.
*/
	// Load up map values for waypoint's x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;

	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";

	// The max s value before wrapping around the track back to 0
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
		map_waypoints_x.push_back(x);
		map_waypoints_y.push_back(y);
		map_waypoints_s.push_back(s);
		map_waypoints_dx.push_back(d_x);
		map_waypoints_dy.push_back(d_y);
	}

	// start off in lane 1
	int lane = 1;

	// a reference velocity to target
	// start at zero to prevent a cold start
	double ref_vel = 0;

	h.onMessage(
			[&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy, &lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
					uWS::OpCode opCode) {
				// "42" at the start of the message means there's a websocket message event.
				// The 4 signifies a websocket message
				// The 2 signifies a websocket event
				//auto sdata = string(data).substr(0, length);
				//cout << sdata << endl;
				if (length && length > 2 && data[0] == '4' && data[1] == '2') {

					auto s = hasData(data);

					if (s != "") {
						auto j = json::parse(s);

						string event = j[0].get<string>();

						if (event == "telemetry") {
							// j[1] is the data JSON object

              /*
              ["x"] The car's x position in map coordinates
              ["y"] The car's y position in map coordinates
              ["s"] The car's s position in frenet coordinates
              ["d"] The car's d position in frenet coordinates
              ["yaw"] The car's yaw angle in the map
              ["speed"] The car's speed in MPH
              */
							// Main car's localization Data
							double car_x = j[1]["x"];
							double car_y = j[1]["y"];
							double car_s = j[1]["s"];
							double car_d = j[1]["d"];
							double car_yaw = j[1]["yaw"];
							double car_speed = j[1]["speed"];

							// Previous path data given to the Planner
							vector<double> previous_path_x = j[1]["previous_path_x"];
							vector<double> previous_path_y = j[1]["previous_path_y"];

							// Previous path's end s and d values
							double end_path_s = j[1]["end_path_s"];
							double end_path_d = j[1]["end_path_d"];

							// Sensor Fusion Data, a list of all other cars on the same side of the road.
							vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];


							int prev_size = previous_path_x.size();

							if(prev_size > 0) {
								car_s = end_path_s;
							}

							bool too_close = false;


							// go through the sensor fusion data

              // for other cars:
              // id, x, y, vx, vy, s , d
							if(sensor_fusion.size() > 0) {

                //for every other car
								for(int i=0; i < sensor_fusion.size(); ++i) {  
									float d = sensor_fusion[i][6]; // lateral distance

									// car is in my lane
                  // lane width is 4, suppose car is in the middle of lane
									if((d < (2 + 4*lane + 2)) && (d > (2 + 4*lane - 2))) {
										double vx = sensor_fusion[i][3];
										double vy = sensor_fusion[i][4];
										double check_speed = sqrt(vx*vx + vy *vy);
										double check_car_s = sensor_fusion[i][5];

										// Using speed we can predict where the car will be in the future

										// If using previous points, we can project s value out

                    // reach point every 0.02 s, prev_size * 0.02 is the total time of previous path
										check_car_s += ((double) prev_size * 0.02 * check_speed);

										// Check for cars in front of us
										// Check for cars in a small window in front of us
										if((check_car_s > car_s)  && (check_car_s - car_s) < 30) {

											// Do some logic here, lower reference velocity so we don't crash into the car
											// in front of us. We could also flag to try to change lanes
											too_close = true;

											std::cout << "SLOWING DOWN" << std::endl;

                      // if possible, change lane
											isLaneChangePossible(sensor_fusion, car_x, car_y, car_s, car_d, car_yaw, car_speed, lane, prev_size, previous_path_x, previous_path_y, map_waypoints_s, map_waypoints_x, map_waypoints_y, ref_vel);
										}
									}
								}
							}

							// gradually reduce or build up the velocity
							// to prevent sudden changes in acceleration
							if(too_close) {
								ref_vel -= 0.224;
							} else if(ref_vel < 49.5) {
								ref_vel += 0.224;
							}

							// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m
							// Later we will interpolate these waypoints with a spline 
              //and fill it in with more points that control speed
							vector<double> ptsx;
							vector<double> ptsy;

							// reference x,y,yaw states
							// Either we will reference the starting point as where the car is at 
              //or the previous paths end point
							double ref_x = car_x;
							double ref_y = car_y;
							double ref_yaw = deg2rad(car_yaw);

							// if the previous path is almost empty, use the car as starting reference
							if(prev_size < 2) {
								// Use 2 points that make the point tangent to the car

								double prev_car_x = car_x - cos(car_yaw);
								double prev_car_y = car_y - sin(car_yaw);

								ptsx.push_back(prev_car_x);
								ptsx.push_back(ref_x);

								ptsy.push_back(prev_car_y);
								ptsy.push_back(ref_y);
							}
							// else, use the previous path's end point as starting reference
							else {
								ref_x = previous_path_x[prev_size - 1];
								ref_y = previous_path_y[prev_size - 1];

								double ref_x_prev = previous_path_x[prev_size - 2];
								double ref_y_prev = previous_path_y[prev_size - 2];
								ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

								// Use 2 points that make the path
								// tangential to the previous path's endpoints
								ptsx.push_back(ref_x_prev);
								ptsx.push_back(ref_x);

								ptsy.push_back(ref_y_prev);
								ptsy.push_back(ref_y);
							}

							// In Frenet, add evenly 30m spaced points ahead of the starting reference
              //then transform the waypoints from s,d to x, y in car's coordinate system
							vector<double> next_wp0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
							vector<double> next_wp2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

							ptsx.push_back(next_wp0[0]);
							ptsx.push_back(next_wp1[0]);
							ptsx.push_back(next_wp2[0]);

							ptsy.push_back(next_wp0[1]);
							ptsy.push_back(next_wp1[1]);
							ptsy.push_back(next_wp2[1]);

          
							for(int i=0; i<ptsx.size(); ++i) {
								double shift_x = ptsx[i] - ref_x;
								double shift_y = ptsy[i] - ref_y;
              
                // transform to globla coordinate from car's coordinate
								ptsx[i] = (shift_x * cos(0 - ref_yaw)) - (shift_y * sin(0 - ref_yaw));
								ptsy[i] = (shift_x * sin(0 - ref_yaw)) + (shift_y * cos(0 - ref_yaw));
							}

							// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
							tk::spline s;
							s.set_points(ptsx, ptsy);
							json msgJson;
							vector<double> next_x_vals;
							vector<double> next_y_vals;

							// Add on all unused points from the previous path onto the generated trajectory
							for(int i=0; i<previous_path_x.size(); ++i) {
								next_x_vals.push_back(previous_path_x[i]);
								next_y_vals.push_back(previous_path_y[i]);
							}

							// calculate how we break up spline points so that we travel at our desired reference velocity

              double target_x = 30.0; // x move 30  in car's coordinate
              
							double target_y = s(target_x);

							// Distance from the origin(where the car is currently present)
							// To the desired horizon point
							double target_distance = sqrt((target_x * target_x) + (target_y * target_y));

							double x_add_on = 0;

							// Number of steps til the horizon
							double N = target_distance/(0.02 * ref_vel/2.24);

							// We will only need to generate points to make the total up to 50
							for(int i=1; i <= 50-previous_path_x.size(); ++i) {
								double x_point = x_add_on + (target_x)/N;
								double y_point = s(x_point);

								x_add_on = x_point;

								double x_ref = x_point;
								double y_ref = y_point;

								// Transform back to global coordinate system from the
								// car's coordinate system
								x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
								y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

								x_point += ref_x;
								y_point += ref_y;

								next_x_vals.push_back(x_point);
								next_y_vals.push_back(y_point);
							}


							// END

							msgJson["next_x"] = next_x_vals;
							msgJson["next_y"] = next_y_vals;

							auto msg = "42[\"control\","+ msgJson.dump()+"]";

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