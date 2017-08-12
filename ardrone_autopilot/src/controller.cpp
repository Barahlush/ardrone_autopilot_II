#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "features.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>


class Circle {
 public:
    float x, y, width, height;
    bool inTheBox;
    
    Circle() {}

    Circle(float _x, float _y, float _width, float _height, bool b):
        x(_x), 
        y(_y), 
        width(_width), 
        height(_height), 
        inTheBox(b)
         {}
};

// Circle output
std::ostream &operator<<(std::ostream &os, Circle& c) { 
    std::string output;
    output = "X: " + std::to_string(c.x) + '\n';
    output += "Y: " + std::to_string(c.y) + '\n';
    output += "Width: " + std::to_string(c.width) + '\n';
    output += "Height: " + std::to_string(c.height) + '\n';
    output += "InTheBox: " + std::to_string(c.inTheBox) + '\n';
    return os << output;
}

class PID {
 public:
    float kP, kD, kI;
    float integralX, integralY;
    float prevErrorX, prevErrorY;
    float dt;

    PID(float Dt, float Kp, float Ki, float Kd): 
        dt(Dt),
        kP(Kp), 
        kD(Kd), 
        kI(Ki), 
        prevErrorX(0),
	    prevErrorY(0), 
        integralX(0),
        integralY(0) {}
   
    float calculate(float error, bool x) {
	float prevError, integral;
	x ? prevError = prevErrorX : prevError = prevErrorY; 
	x ? integral = integralX : integral = integralY; 
        float outP = kP * error;

        integral += error * dt;
        float outI = kI * integral;
        
        float derivative;

        if (prevError != 0) {
            derivative = (error - prevError) / dt;
        } else {
            derivative = 0;
        }

        float outD = kD * derivative;

	x ? integralX = integral : integralY = integral; 
        float output = outP + outI + outD;

    std::cout << "--------------PID------------------\n";
    std::cout << "Error: " << error << '\n';
    std::cout << "prevError: " << prevError << '\n';
    
    std::cout << "dt: : " << dt << '\n';
    std::cout << "IntegralX: " << integralX << '\n';
    std::cout << "IntegralY: " << integralY << '\n';

	std::cout << "kP: " << kP << "| P: " << outP << '\n';
	std::cout << "kI: " << kI << "| I: " << outI << '\n';
	std::cout << "kD: " << kD << "| D: " << outD << '\n';
	if (x)
        prevErrorX = error;
	else
        prevErrorY = error;

        return output;
    }        
};


// Controller struct

class ControlCenter
{
    public:
	int counter;
        bool enabled, xInBox, yInBox;//, boxCaptured;
        float imgRows, imgCols;
        float triCenterX, triCenterY;
        Box box;
        std::vector<Circle> targ;
        ros::Publisher cmdPublisher;
        PID pid;
        ros::Time lastLoop;
        ControlCenter(): pid(0.06, 0.07, 0, 0.02) {} 
};

struct ControlCenter control;


// Enable/disable controller

void onEnableCtrl(const std_msgs::Empty& toggle_msg) {
    control.enabled = !control.enabled;
    if (control.enabled)
        std::cout << "Target autopilot enabled.\n";
    else
        std::cout << "Target autopilot disabled.\n"; 
}


// Extract information from incoming message 

void parseArray(const std_msgs::Float32MultiArray& msg) {
    if (msg.data.size() != 0) {
        size_t i = 0;
        Circle circle;
        while (i < msg.data.size()) {
            circle.x = msg.data[i++];
            circle.y = msg.data[i++];
            circle.width = msg.data[i++];
            circle.height = msg.data[i++];
            circle.inTheBox = !msg.data[i++];
            control.targ.push_back(Circle(circle));
        }   
    }   
}

bool centerInTriangle() {
    float x0 = (control.box.right+control.box.left) / 2;
    float y0 = (control.box.top+control.box.bottom) / 2;
    float x1 = control.targ[0].x,
          x2 = control.targ[1].x,
          x3 = control.targ[2].x,
          y1 = control.targ[0].y,
          y2 = control.targ[1].y,
          y3 = control.targ[2].y;
    float p1 = (x1 - x0) * (y2 - y1) - (x2 - x1) * (y1 - y0);
    float p2 = (x2 - x0) * (y3 - y2) - (x3 - x2) * (y2 - y0);
    float p3 = (x3 - x0) * (y1 - y3) - (x1 - x3) * (y3 - y0);
    if ((p1 >= 0 && p2 >= 0 && p3 >= 0) || (p1 <= 0 && p2 <= 0 && p3 <= 0))
    {
        return true;
    } else {
        return false;
    }
}

bool checkTheBox() {
    for (auto& circle : control.targ)
        if (!circle.inTheBox) {
            if (circle.x > control.box.right || circle.x < control.box.left)
                control.xInBox = false;
            if (circle.y > control.box.bottom || circle.y < control.box.top)
                control.yInBox = false;
            return false;
        }
    return true;
}


// Center of rectangle around the triangle

void getTriangleCenter() {
    float maxX = 0, minX = control.box.right + control.box.left,
          maxY = 0, minY = control.box.top + control.box.bottom;
  
    for (auto& circle : control.targ) {
        if (circle.x > maxX)
            maxX = circle.x;
        if (circle.x < minX)
            minX = circle.x;
        if (circle.y > maxY)
            maxY = circle.y;
        if (circle.y < minY)
            minY = circle.y;
    }
    control.triCenterX = (maxX + minX) / 2;
    control.triCenterY = (maxY + minY) / 2;
}
    
void controller(geometry_msgs::Twist& msg) {

   std::vector<float> errorsX;
    std::vector<float> errorsY;
    float middleX = (control.box.right+control.box.left) / 2;
    float middleY = (control.box.top+control.box.bottom) / 2;
/*
    if (!checkTheBox()) {

        float Xerr = control.triCenterX - middleX;
        float Yerr = control.triCenterY - middleY;
        std::cout << "YERR : " << Yerr << " XERR: " << Xerr << "\n"; 

        if (!control.xInBox) {
            if (std::abs(Xerr) > (control.box.right - middleX) / 8) {
                float velX = control.pid.calculate(Xerr / middleX, true);
                msg.linear.y = -velX;
            } else control.xInBox = true;
        } else if (!control.yInBox)
            if (std::abs(Yerr) > (control.box.bottom - middleY) / 8) {
                float velY = control.pid.calculate(Yerr / middleY, false);
                msg.linear.x = -velY;
            } else control.yInBox = true;
    }

 */   for (const auto& circle : control.targ) {

        if (!circle.inTheBox) {
            
        float Xerr = circle.x - middleX;
		/*float Xerr = 0;
		if (circle.x > control.box.right)
		    Xerr = circle.x - control.box.right;
		else if (circle.x < control.box.left) {
		    Xerr = circle.x - control.box.left;
		}*/
                errorsX.push_back(Xerr);

        float Yerr = circle.y - middleY;
		/*float Yerr = 0;
		if (circle.y > control.box.bottom)
		    Yerr = circle.y - control.box.bottom;
		else if (circle.y < control.box.top) {
		    Yerr = circle.y - control.box.top;
		}*/
                errorsY.push_back(Yerr);
                
                std::cout << "YERR : " << Yerr << " XERR: " << Xerr << "\n"; 
        }
    }
    if (errorsX.size() != 0) {
        float summError = 0, avError;
        size_t numError = errorsX.size();
        for (const auto& error : errorsX) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.left;// middleX;
//	control.pid.kP = std::max((float)0.05, (320 - control.targ[0].width) / 1800);
        float vel = control.pid.calculate(avError, true);
        msg.linear.y = -vel;
    }
    if (errorsY.size() != 0) {
        float summError = 0, avError;
        size_t numError = errorsY.size();
        for (const auto& error : errorsY) {
            summError += error;
            ++numError;
        }
        avError = (summError / numError) / control.box.top;//middleY;
//	control.pid.kP = std::max((float)0.1, (320 - control.targ[0].height) / 300);
        float vel = control.pid.calculate(avError, false);
        msg.linear.x = -vel;
    }

    std::cout << "Command sended!\n";
    std::cout << "Vx -> " << msg.linear.y << '\n';
    std::cout << "Vy -> " << msg.linear.x << '\n';
}

// Recieving information and controlling the drone
void onTarget(const std_msgs::Float32MultiArray& msg) {
        if (control.enabled) {
            geometry_msgs::Twist message;
		if ((ros::Time::now() - control.lastLoop).toSec() >= 0.06) {
            control.targ.clear();

            parseArray(msg);
                
            control.pid.dt = (ros::Time::now() - control.lastLoop).toSec();
            control.lastLoop = ros::Time::now();
           // getTriangleCenter();
            controller(message);
            
            // Circle information output
            
            std::cout << "-----------------------\n";
            std::cout << "kP: " << control.pid.kP << ' ';
            std::cout << "kI: " << control.pid.kI << ' ';
            std::cout << "kD: " << control.pid.kD << '\n' << '\n';

            std::cout << "-----------------------\n";
	    std::cout << "BOX LEFT: " << control.box.left << '\n';
	    std::cout << "BOX RIGHT: " << control.box.right << '\n';
	    std::cout << "BOX TOP: " << control.box.top << '\n';
	    std::cout << "BOX BOTTOM: " << control.box.bottom << '\n' << '\n';
            std::cout << "-----------------------\n";
            for (auto& circle : control.targ) {
                std::cout << circle << '\n';
            }
            std::cout << "-----------------------\n";
        control.cmdPublisher.publish(message);
        }/* else if (control.counter == 50){
			control.counter = 0;
		} else {
			++control.counter;
		}*/
			
	    
	} else {
        control.pid.integralX = 0;
        control.pid.integralY = 0;
        control.pid.prevErrorX = 0;
        control.pid.prevErrorY = 0;
    }
}

// Extract box information from incoming message

void onBox(const std_msgs::Float32MultiArray& msg) {
    //if (!control.boxCaptured) {
        control.box.left = msg.data[0];
        control.box.right = msg.data[1];
        control.box.top = msg.data[2];
        control.box.bottom = msg.data[3];
        control.imgRows = msg.data[4];
        control.imgCols = msg.data[5];
       // control.boxCaptured = true;
    //}
}
        
void onPidD(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP += 0.005;
                  break;
        case 'i': control.pid.kI += 0.001;
                  break;
        case 'd': control.pid.kD += 0.005;
                  break;
    }
}

void onPidI(const std_msgs::String& str) {
    switch (str.data[0]) {
        case 'p': control.pid.kP -= 0.0005;
                  break;
        case 'i': control.pid.kI -= 0.0005;
                  break;
        case 'd': control.pid.kD -= 0.0005;
                  break;
    }
}


int main(int argc, char **argv)
{
    /* 
     * Vasya was here
     */

    ros::init(argc, argv, "controller");
    ros::NodeHandle node;
    
    ros::Subscriber boxSub = 
            node.subscribe("box", 1, onBox);

    ros::Subscriber enableSub = 
            node.subscribe("controller/enable", 5, onEnableCtrl);


    ros::Subscriber targetSub = 
            node.subscribe("target", 5, onTarget);

    ros::Subscriber pidDecreaseSub = 
            node.subscribe("pid/decrease", 5, onPidD);

    ros::Subscriber pidIncreaseSub = 
            node.subscribe("pid/increase", 5, onPidI);

    control.cmdPublisher =
            node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    
    ros::spin();

    return 0;
}
