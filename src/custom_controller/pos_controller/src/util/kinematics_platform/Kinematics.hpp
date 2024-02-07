#ifndef Kinematics_H
#define Kinematics_H

#include <math.h>
#include <array>

//constants
#define A 0
#define B 1
#define C 2

class Machine { //machine class
    public:
        //Library Content
        Machine(double _d, double _e, double _f, double _g) {
            d = _d;
            e = _e;
            f = _f;
            g = _g;
            deviation = 0.1f;
        }

        double theta(int leg, double hz, double nx, double ny) {
            //create unit normal vector
            nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1);  //magnitude of the normal vector
            nx /= nmag;
            ny /= nmag;
            nz = 1 / nmag;
            //calculates angle A, B, or C
            switch (leg) {
                case A:  //Leg A
                    y = d + (e / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))));
                    z = hz + e * ny;
                    mag = sqrt(pow(y, 2) + pow(z, 2));
                    angle = acos(y / mag) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                    break;
                case B:  //Leg B
                    x = (sqrt(3) / 2) * (e * (1 - (pow(nx, 2) + sqrt(3) * nx * ny) / (nz + 1)) - d);
                    y = x / sqrt(3);
                    z = hz - (e / 2) * (sqrt(3) * nx + ny);
                    mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                    angle = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                    break;
                case C:  //Leg C
                    x = (sqrt(3) / 2) * (d - e * (1 - (pow(nx, 2) - sqrt(3) * nx * ny) / (nz + 1)));
                    y = -x / sqrt(3);
                    z = hz + (e / 2) * (sqrt(3) * nx - ny);
                    mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                    angle = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                    break;
            }
            return (angle * (180 / M_PI));  //converts angle to degrees and returns the value
        }

        std::array<double, 2> inverse_theta(int leg, double hz, double angle_inp) {
        // void Machine::inverse_theta(int leg, double hz, double angle_inp) {
            std::array<double, 2> nx_ny_output;
            nx_ny_output[0] = -1000.f;
            nx_ny_output[1] = -1000.f;
            double angle_res = -1000.f;
            for (double i = -0.3000f; i < 0.3000f; i += 0.0001f) {
                for (double j = -0.3000f; j < 0.3000f; j += 0.0001f) {
                    double nx = i;
                    double ny = j;
                    //create unit normal vector
                    nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1);  //magnitude of the normal vector
                    nx /= nmag;
                    ny /= nmag;
                    nz = 1 / nmag;
                    //calculates angle A, B, or C
                    switch (leg) {
                        case A:  //Leg A
                            y = d + (e / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))));
                            z = hz + e * ny;
                            mag = sqrt(pow(y, 2) + pow(z, 2));
                            angle_res = acos(y / mag) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                            break;
                        case B:  //Leg B
                            x = (sqrt(3) / 2) * (e * (1 - (pow(nx, 2) + sqrt(3) * nx * ny) / (nz + 1)) - d);
                            y = x / sqrt(3);
                            z = hz - (e / 2) * (sqrt(3) * nx + ny);
                            mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                            angle_res = acos((sqrt(3) * x + y) / (-2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                            break;
                        case C:  //Leg C
                            x = (sqrt(3) / 2) * (d - e * (1 - (pow(nx, 2) - sqrt(3) * nx * ny) / (nz + 1)));
                            y = -x / sqrt(3);
                            z = hz + (e / 2) * (sqrt(3) * nx - ny);
                            mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                            angle_res = acos((sqrt(3) * x - y) / (2 * mag)) + acos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
                            break;
                    }
                    if (abs(angle_res - angle_inp) <= deviation) {
                        // std::cout << "found match: " << nx << ", " << ny << std::endl;
                        nx_ny_output[0] = nx;
                        nx_ny_output[1] = ny;
                        return nx_ny_output;
                    }
                }
            }
            return nx_ny_output;
            // return (angle * (180 / M_PI));  //converts angle to degrees and returns the value
        }

    private:
        double deviation;
        //Global User Defined Constants
        double d;  //distance from the center of the base to any of its corners
        double e;  //distance from the center of the platform to any of its corners
        double f;  //length of link #1
        double g;  //length of link #2

        //Calculation Variables
        double nmag, nz;  //magnitude and z component of the normal vector
        double x, y, z;   //generic variables for the components of legs A, B, and C
        double mag;       //generic magnitude of the leg vector
        double angle;     //generic angle for legs A, B, and C
};

#endif
