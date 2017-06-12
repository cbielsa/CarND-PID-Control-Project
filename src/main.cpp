
#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>


// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

// CONFIGURABLE PARAMETERS ==================================

// true to optimize steering PID parameters with Twiddle,
// false to drive with fixed parameters
bool optimize = true;

// optimization hyper-parameters used in case optimize:=true
const int iNumCyclesOptimization = 1500;
int iNumParam = 2;  // PD if 2, PDI if 3

// auxiliary variables used in optimization
int iCycle = 0;
double cost = 0.;
double best_cost;
int iParam = 0;  // twiddle param (0 for P, 1 for I, 2 for D)
bool changeParam = true;
int iStage = 1;  // stage of the twiddle algorithm

// parameters of steering PID controller
// (or initial guess in case of optimize:=true)
double Kp = 0.09;
double Ki = 0.;
double Kd = 0.70;

// initial delta-parameters for twiddle optimization
// of steering PID controller, used in case optimize:=true
double dKp = 0.009;
double dKi = 0.00003;
double dKd = 0.07;

// tuning for PD speed controller
// target speed when steering angle is zero
const double maxSpeed = 50;

// minimum target speed (for very large steering angle)
const double minSpeed = 15;

// vectors of parmeters and delta-parameters
// for PID controller of steering angle
std::vector<double> vPDI(iNumParam);
std::vector<double> vdPDI(iNumParam);


int main()
{
  uWS::Hub h;

  // Construct PID objects for steering and throttle control, respectively
  PID pidSteering;
  PID pidThrottle;

  // Fill in value of parameters and delta-parameters vectors
  vPDI[0] = Kp;
	vPDI[1] = Kd;
	if( iNumParam==3 )
		vPDI[2] = Ki;
	vdPDI[0] = dKp;
  vdPDI[1] = dKd;
  if( iNumParam==3 )
  	vdPDI[2] = dKi;

  // Initialize PID controllers
  pidSteering.Init(Kp, Ki, Kd);
  pidThrottle.Init(0.07, 0., 0.005);

  h.onMessage([&pidSteering, &pidThrottle](
  	uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {

        	// case drive without optimization
          // or regular cycle of optimization
          if( !optimize || iCycle < iNumCyclesOptimization ) {

          	// j[1] is the data JSON object
          	double cte = std::stod(j[1]["cte"].get<std::string>());
          	double speed = std::stod(j[1]["speed"].get<std::string>());
          	double angle = std::stod(j[1]["steering_angle"].get<std::string>());

            // speed control ------------------------------------------------------

          	// calculate target speed with an inverse linear relation to steering angle
          	// (for 0 steering angle, target speed is max speed; for 25 deg, target speed is 0)
          	double targetSpeed = maxSpeed*(1-std::fabs(angle)/25.);
          	
          	// apply low hard limit to target speed
          	if( targetSpeed<minSpeed )
          		targetSpeed = minSpeed;

          	// calculate throttle actuation with PD controller applied to
            // difference between measured and target speeds
            pidThrottle.UpdateError(speed-targetSpeed);
          	double throttle = pidThrottle.Actuation();
          	//std::cout << "throttle : " << throttle << ", angle : " << angle << ", targ speed : " << targetSpeed << std::endl;


            // steering angle control ---------------------------------------------

            pidSteering.UpdateError(cte);
            double steer_value = pidSteering.Actuation();
            
            // DEBUG
            //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

            // prepare and send control message to simulator
            json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

            // if running in optimization mode, update cost function
            if( optimize )
            {
              ++iCycle;
              cost += cte*cte;
            }


          // case optimization:=true and end of optimization drive
          } else {

          	// print-out cost
          	cost /= iNumCyclesOptimization;
          	std::cout << std::endl;
          	std::cout << "Run ended" << std::endl;
          	std::cout << "  * cost: " << cost << std::endl;
          	if( iNumParam==3 )
          	{
          		std::cout << "  * PID params: " << vPDI[0] << ", " << vPDI[2] << ", " << vPDI[1] << std::endl;
          		std::cout << "  * Last PID delta: " << vdPDI[0] << ", " << vdPDI[2] << ", " << vdPDI[1] << std::endl;
          	}
          	else
          	{
          		std::cout << "  * PD params: " << vPDI[0] << ", " << vPDI[1] << std::endl;
          	  std::cout << "  * Last PD delta: " << vdPDI[0] << ", " << vdPDI[1] << std::endl;
						}
          	std::cout << std::endl;


    				// adjust PID controller parms with twiddle algorithm ---------------------
    				
          	if( iStage == 1 )  // first optimization run
          	{
          		best_cost = cost;
          		vPDI[iParam] += vdPDI[iParam];  // udpate param

          		// prepare for next run
          		iStage = 2;
          	}

          	else if( iStage == 2 )
          	{

    					if( cost < best_cost )
    					{
    						best_cost = cost;
    						vdPDI[iParam] *= 1.1;  // get more aggresive in next param update

    						// prepare for next run
    						iParam = (iParam+1)%iNumParam;  // cycle over PID param
    						vPDI[iParam] += vdPDI[iParam];  // udpate param
          			iStage = 2;
    					}
    					else
    					{
    						vPDI[iParam] -= 2*vdPDI[iParam];

    						// prepare for next run
    						iStage = 3;
    					}

          	}

          	else
          	{
    					if( cost < best_cost )
    					{
    						best_cost = cost;
    						vdPDI[iParam] *= -1.1;  // change dir and get more aggresive in next param update

    						// prepare for next run
    						iParam = (iParam+1)%iNumParam;  // cycle over PID param
    						vPDI[iParam] += vdPDI[iParam];  // udpate param
          			iStage = 2;
    					}
    					else
    					{
    						vPDI[iParam] += vdPDI[iParam];  // revert param update
    						vdPDI[iParam] *= 0.9;           // get less aggresive in next param update

    						// prepare for next run
    						iParam = (iParam+1)%iNumParam;  // cycle over PID param
    						vPDI[iParam] += vdPDI[iParam];  // udpate param
    						iStage = 2;
    					}

          	}


    				// initialize PID object with null errors and updated params
    				if( iNumParam==2 )
    					pidSteering.Init(vPDI[0], 0., vPDI[1]);
    				else
    					pidSteering.Init(vPDI[0], vPDI[2], vPDI[1]);

    				// reset cycle counter and send resent message to simulator
          	iCycle = 0;
          	cost = 0.;

						std::string msg = "42[\"reset\",{}]";
    				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          }

        }

      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });


  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    //std::cout << "Connected!!!" << std::endl;
  });


  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
