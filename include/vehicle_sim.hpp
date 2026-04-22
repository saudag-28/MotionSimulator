#include <iostream>
#include <string>
#include <string_view>
#include <vector>
#include <fstream>
#include <cmath>

enum direction {
    NORTH = 0,   // M_PI/2
    EAST = 1,    // 0 rad
    SOUTH = 2,   // (3 * M_PI/2)
    WEST = 3     // M_PI
};

struct agent {
    double x;
    double y;
    double heading; // radians
    double speed;   // constant - kmph
    double yawRate; // constant - radians/second
    direction dir;
};


class Simulator {

    private:
        double dt = 0.5;  // simulation timestep

    public:

        double getdt() const {
            return dt;
        }

        // straight line motion (velocity model) - 2 axis
        void getNextState(agent& a) {
            a.x = a.x + (a.speed * dt);
            a.y = a.y + (a.speed * dt);
        }

        // get next state with heading
        void getNextStateHeading(agent&a) {
            a.x = a.x + (a.speed * cos(a.heading) * dt);
            a.y = a.y + (a.speed * sin(a.heading) * dt);
        }

        // get nextDir
        void nextDir(agent& a) {
            a.dir = static_cast<direction>((a.dir - 1 + 4) % 4);
        }

        // update heading based on the current heading direction
        void nextHeading(agent& a) {
            if (a.dir == 0) {
                a.heading = M_PI/2.0;
            } else if (a.dir == 1) {
                a.heading = 0;
            } else if (a.dir == 2) {
                a.heading = 3.0 * M_PI/2.0;
            } else {
                a.heading = M_PI;
            }
        }

        // change heading direction without changing the spatial (x,y) location
        void inplaceRotate(agent& a) {
            a.heading = a.heading + (a.yawRate * dt);
        }

        /// @brief Forward (explicit) Euler integration of the unicycle model. 
        ///     You’re numerically integrating: x˙= vcosθ, y˙​= vsinθ, θ˙= ω, with: 
        ///     xk+1 ​= xk​ + vcos(θk​)dt, yk+1 ​= yk ​+ vsin(θk​)dt, θk+1 ​= θk ​+ ωdt
        ///     This is simple and works well for small dt, but it approximates the true circular arc with straight segments, 
        ///     so you’ll see small drift unless dt is very small.
        /// Analytical (exact) update
        /// If velocity v and yaw rate ω are constant over the timestep, you can integrate exactly over dt. The motion follows a circular arc, and the closed-form update is:
        /// xk+1 ​= xk​ + (v/w)​[sin(θk​+ωdt) − sin(θk​)], yk+1 ​= yk​ + (v/w)​[cos(θk​+ωdt) − cos(θk​)], θk+1 ​= θk ​+ ωdt
        /// ✅ Exact circular motion (no polygon approximation)
        /// ✅ Conserves the radius r= v/w
        /// ✅ No accumulated integration drift
        /// ✅ Works well even with larger dt
        /// Intuition
        /// Euler: “move straight, then turn a bit” → polygon
        /// Analytical: “follow the arc directly” → perfect circle
        void circleRotate(agent& a) {
            /// the x and y updates here are wrong because i am completely ignoring speed and dt. 
            /// these updates just say - "move 0.1 units per step"
            /// but we need, Euler - "move v.dt units per step"
            /// if more precision, Analytical - "follow exact arc of a circle"
            // a.x = a.x + 0.1*cos(a.heading);
            // a.y = a.y + 0.1*sin(a.heading);
            // a.heading = a.heading + (a.yawRate * dt);

            a.x = a.x + a.speed*cos(a.heading)*dt;
            a.y = a.y + a.speed*sin(a.heading)*dt;
            a.heading = a.heading + (a.yawRate * dt);
        }

        // analytical form which uses SE(2)
        void updatePoseExact(agent& a) {
            if (a.yawRate < 1e-6) {
                circleRotate(a);
            } 
            else {
                a.x = a.x + (a.speed/a.yawRate) * (sin(a.heading + a.yawRate*dt) - sin(a.heading));
                a.y = a.y + (a.speed/a.yawRate) * (cos(a.heading + a.yawRate*dt) - cos(a.heading));
                a.heading = a.heading + (a.yawRate * dt);
            }
        }
};