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
#include "Vehicle.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

int main() {
  uWS::Hub h;

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
  double ref_val = 0;
  int lane = 1;
  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy,&ref_val,&lane/*&V*/]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
          int path_size = previous_path_x.size();
          bool tooclose = false;
       
          if(path_size > 0)
          {
            car_s = end_path_s;
          }

          for(int i = 0; i < sensor_fusion.size(); i++)
          {
            float d = sensor_fusion[i][6];
            bool foundleft = false;
            bool foundright = false;
            //double costleft = 0.0;
            //double costright = 0.0;
            vector<double> sl_list;
            vector<double> sr_list;

            //Check if vehicle ahead too close 
            if((4*lane < d) && (d < 4*(lane+1)))
            {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double v = sqrt(vx*vx + vy*vy);
              double s = sensor_fusion[i][5];
              s += ((double)path_size*(v*0.02));
              //std::cout<<"\n"<<" closest car position "<<s<<" car_s "<<car_s<<" car_d "<<car_d;
              
              if ((s > car_s) && (s - car_s < 40))
              {
                tooclose = true; 
                //std::cout<<"\n Change left lane ";
                for(int i = 0 ; i < sensor_fusion.size();i++)
                {
                  double dl = sensor_fusion[i][6];
                  if((4*lane-4 < dl)&&(dl < 4*lane)&&(4*lane < car_d)&&(car_d < 4*(lane+1)))
                  {
                    double vxl = sensor_fusion[i][3];
                    double vyl = sensor_fusion[i][4];
                    double vl = sqrt(vxl*vxl + vyl*vyl); 
                    double sl = sensor_fusion[i][5];
                    sl += ((double)path_size*0.02* vl);
                    //std::cout<<"\n"<<"left lane car "<<sl<<"car_s "<<car_s<<"car_d "<<car_d;
                    sl_list.push_back(sl);
                  }
                }

                int count = 0; 
                for(int i = 0 ; i < sl_list.size();i++)
                {
                  //check if good gap exists between the ego vehicle and left lane vehicles
                  if(((sl_list[i] > car_s) && (sl_list[i]-(car_s) > 30))
                    ||((sl_list[i] < car_s) && (car_s-sl_list[i]) > 30))
                  {
                    foundleft = true;
                    count += 1;
                  }
                }

                if((count == sl_list.size()) && (foundleft==true))
                { 
                    //costleft = abs(car_speed-vl)/(car_speed);
                    //int oldlane = lane;
                    lane = lane - 1;
                    //std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                }   
                  
                if ((!foundleft))
                {
                  //std::cout<<"\n"<<" Change right lane ";
                  for(int i = 0 ; i < sensor_fusion.size();i++)
                  {
                    double dr = sensor_fusion[i][6];
                    if((4*(lane+2)>dr)&&(dr > 4*(lane+1))&&(4*(lane+1) > car_d)&&(car_d > 4*lane))
                    {                  
                      double vxr = sensor_fusion[i][3];
                      double vyr = sensor_fusion[i][4];
                      double vr = sqrt(vxr*vxr + vyr*vyr);
                      double sr = sensor_fusion[i][5];
                      sr += ((double)path_size*0.02* vr);
                      //std::cout<<"\n"<<"right lane car "<<sr<<"car_s "<<car_s<<"car_d "<<car_d;
                      sr_list.push_back(sr);
                    }}
                  
                    int count = 0;
                    for(int i = 0 ; i < sr_list.size();i++)
                    {
                      //check if good gap exists between the ego vehicle and right lane vehicles
                      if(((sr_list[i] > car_s) && (sr_list[i]-(car_s) > 30))
                        ||((car_s > sr_list[i]) && ((car_s - sr_list[i]) > 30)))
                      {
                        foundright = true;
                        count += 1;
                      }
                    }
                  
                    if((count == sr_list.size()) && (foundright==true))
                    {   
                      //costright = abs(car_speed - vr)/(car_speed);
                      //int oldlane = lane;
                      lane = lane + 1;
                      //std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                    }
                  }
                }
              }
            }  
#if 0
                if((foundleft == true) && (foundright == true) && (costleft > costright)) 
                  {
                    int oldlane = lane;
                    lane = lane + 1;
                    ref_val = velright;
                    std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                    //V.state = KEEP_LANE;
                  }
                else if((foundleft == true) && (foundright == true) && (costleft < costright))
                  {
                    int oldlane = lane;
                    lane = lane - 1;
                    ref_val = velleft;
                    std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                    //V.state = KEEP_LANE;
                  }
                else if (foundleft == true) 
                  {
                    int oldlane = lane;
                    lane = lane - 1;
                    ref_val = velleft;
                    std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                  }

                else if (foundright == true) 
                  {
                    int oldlane = lane;
                    lane = lane + 1;
                    ref_val = velright;
                    std::cout<<"Lane Changed from "<<oldlane<<"to "<<lane;
                  }
                else
                  {
                    std::cout<<"Not possible "<<"\n";
                  }
#endif                        
          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;
          vector<double> ptsx;
          vector<double> ptsy;
          
          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          /* Fill anchor points */
          if(tooclose)
          {
            ref_val -= 0.224;
          }
          else if(ref_val < 49.5)
          {
            ref_val += 0.224;
          }

          double  ref_x = car_x;
          double  ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if(path_size < 2)
          {
            ptsx.push_back(car_x);
            ptsy.push_back(car_y);
            ptsx.push_back(car_x-cos(ref_yaw));
            ptsy.push_back(car_y-sin(ref_yaw));
            std::cout<<car_x<<" "<<car_y<<" "<<"\n";
          }
          else
          {
            ref_x = previous_path_x[path_size-1];
            ref_y = previous_path_y[path_size-1];
            ptsx.push_back(ref_x);
            ptsy.push_back(ref_y);

            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];

            ptsx.push_back(pos_x2);
            ptsy.push_back(pos_y2);

            ref_yaw = atan2(ref_y-pos_y2, ref_x-pos_x2);
          } 

          vector<double> xy;
          xy = getXY(car_s+30,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);
          xy = getXY(car_s+60,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);
          xy = getXY(car_s+90,2+4*lane,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          ptsx.push_back(xy[0]);
          ptsy.push_back(xy[1]);


          for(int i = 0; i < ptsx.size(); i++)
          {
            double shift_x = ptsx[i]-ref_x;
            double shift_y = ptsy[i]-ref_y;

            ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
            ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
          }

          /* Fill future points */
          
          tk::spline s;
          s.set_points(ptsx, ptsy);
          double s_x = 30;
          double s_y = s(s_x);
          double target_dist = sqrt((s_x*s_x) + (s_y*s_y));
          double x_add_on = 0;
          for( int i = 0; i < previous_path_x.size(); i++)
          {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          for(int i = 1;i<=50-previous_path_x.size();i++)
          {
              double N = (target_dist/(0.02*ref_val/2.24));
              double x_point = x_add_on+(s_x/N);
              double y_point = s(x_point);
              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;

              x_point = (x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw));
              y_point = (x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
          }

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

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