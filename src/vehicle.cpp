#include "vehicle.h"

using namespace std;

Vehicle::Vehicle(){
  this->id = -1;
  this->stopped = true;
  this->state = STATE::KEEP_LANE;
}

Vehicle::Vehicle(int id, double x, double y, double v, double s, double d){
  this->id = id;
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
}

/* GETTERS*/
int Vehicle::get_id(){
  return this->id;
}

double Vehicle::get_x(){
  return this->x;
}

double Vehicle::get_y(){
  return this->y;
}

double Vehicle::get_v(){
  return this->v;
}

double Vehicle::get_s(){
  return this->s;
}

double Vehicle::get_d(){
  return this->d;
}

STATE Vehicle::get_state(){
  return this->state;
}

LANE Vehicle::lane(){
  LANE lane;
  if (this->d < 4.0) {
    lane = LANE::LEFT;
  }
  else if ((this->d >= 4.0) && (this->d < 8.0)) {
    lane = LANE::CENTER;
  }
  else {
    lane = LANE::RIGHT;
  }
  return lane;
}

void Vehicle::update_vehicle_state(double x, double y, double v, double s, double d, double yaw){
  this->x = x;
  this->y = y;
  this->v = v;
  this->s = s;
  this->d = d;
  this->yaw = yaw;
}

bool Vehicle::is_stopped(){
  return this->stopped;
}

void Vehicle::lock_start(){
  this->stopped = false;
}

void Vehicle::set_previous_s(vector<double> previous_s){
  this->previous_s = previous_s;
}

void Vehicle::set_previous_d(vector<double> previous_d){
  this->previous_d = previous_d;
}

vector<double> Vehicle::prev_s(){
  return this->previous_s;
}

vector<double> Vehicle::prev_d(){
  return this->previous_d;
}

void Vehicle::set_state(STATE state){
  this->state = state;
}
