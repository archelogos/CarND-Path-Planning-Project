#ifndef UTILS_H
#define UTILS_H

#include <fstream>
#include <math.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include <string>

using namespace std;

const double TRACK_DISTANCE = 6945.554; //m
const double ROAD_WIDTH = 12.0; //m
const double SPEED_LIMIT = 20.0; //m/s
const double AT = 0.02; //s
const double CYCLES = 2;
const double POINTS = 50;
const double SAFETY_DISTANCE = 40.0; //m
const double GUARD_DISTANCE = 20.0; //m

enum class LANE { LEFT, CENTER, RIGHT };

enum class STATE { START, KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };

#endif // UTILS_H
