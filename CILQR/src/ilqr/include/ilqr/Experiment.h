#pragma once

#include <vector>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ilqr/VehicleArray.h> // Replace with your package name
#include <ilqr/Vehicle.h>      // Replace with your package name

bool isCollision(const ilqr::Vehicle &v1, const ilqr::Vehicle &v2);