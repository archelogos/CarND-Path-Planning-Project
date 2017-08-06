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

const double TRACK_DISTANCE = 6945.554;
const double ROAD_WIDTH = 12.0;
const double SPEED_LIMIT = 20.0;
const double AT = 0.02; //s
const double CYCLES = 2;
const double POINTS = 50; //points per each s
const int MIN_PATH_POINTS = 50;
const double SAFETY_DISTANCE = 40.0;

/* LANE */
enum class LANE { LEFT, CENTER, RIGHT };

enum class STATE { START, KEEP_LANE, CHANGE_LEFT, CHANGE_RIGHT };

#endif // UTILS_H
