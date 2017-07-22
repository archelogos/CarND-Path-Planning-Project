#ifndef VEHICLE_H
#define VEHICLE_H

class Vehicle {
  public:
    Vehicle(int id, double x, double y, double vx, double vy, double s, double d) {
      this->id = id;
      this->x = x;
      this->y = y;
      this->vx = vx;
      this->vy = vy;
      this->s = s;
      this->d = d;
    };
    ~Vehicle() {};

    /* ["sensor_fusion"] A 2d vector of cars and then that car's [
    id: car's unique ID,
    x: car's x position in map coordinates,
    y: car's y position in map coordinates,
    vx: car's x velocity in m/s,
    vy: car's y velocity in m/s,
    s: car's s position in frenet coordinates,
    d: car's d position in frenet coordinates. */

    int id;
    double x;
    double y;
    double vx;
    double vy;
    double s;
    double d;

};

#endif // VEHICLE_H
