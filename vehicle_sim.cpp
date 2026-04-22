// 1. Program to move robot in a straight line - done
// 2. Program to move robot in a square shape - done
// 3. Program to rotate robot in place - done
// 4. Program to rotate robot in a circle - done
// 5. two agents moving, check collision
//      a. timestep based check
//      b. collision model based check

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

int main(int argc, char* argv[]) {

    for (int i = 1; i < argc; ++i) {
        std::string_view arg(argv[i]);

        if (arg == "1") {
            std::ofstream file("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/singleAxis.csv");
            agent a1;
            a1.x = 0;
            a1.y = 1;
            a1.speed = 1;

            Simulator xAxis;
            
            for (int t = 0; t < 20; ++t) {

                xAxis.getNextState(a1);

                file << a1.x << "," << a1.y << "\n";
            }
        }

        if (arg == "2") {
            std::ofstream file("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/squareShape.csv");
            agent a1;
            a1.x = 0;
            a1.y = 1;
            a1.speed = 1;
            a1.dir = EAST;
            a1.heading = 0.0;

            Simulator square;
            
            for (int t = 0; t < 60; ++t) {
                
                if (t % 10 == 0) {
                    square.nextDir(a1);
                    // update heading
                    square.nextHeading(a1);
                }

                square.getNextStateHeading(a1);

                file << a1.x << "," << a1.y << "," << a1.heading << "\n";
            }
        }

        if (arg == "3") {
            std::ofstream file("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/inplace.csv");

            agent a1;
            a1.x = 0;
            a1.y = 1;
            a1.speed = 1;
            a1.dir = EAST;
            a1.heading = 0.0;

            Simulator square;
            
            for (int t = 0; t < 60; ++t) {
                
                if (t % 10 == 0) {
                    square.nextDir(a1);
                    // update heading
                    square.nextHeading(a1);
                }

                square.inplaceRotate(a1);

                file << a1.x << "," << a1.y << "," << a1.heading << "\n";
            }
        }

        if (arg == "4") {
            std::ofstream file("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/circle.csv");

            agent a1;
            a1.x = 10;
            a1.y = 0;
            a1.speed = 4.0;   // to get radius of _, r = v/w = speed/yawRate
            a1.yawRate = 2.0;
            a1.heading = 0.0;
            
            Simulator square;

            // total steps required to complete full circle
            double total_time = 2*M_PI/a1.yawRate;
            double total_steps = static_cast<int>(std::round(total_time/square.getdt()));

            for (int t = 0; t < total_steps; ++t) {
                
                // square.circleRotate(a1);
                square.updatePoseExact(a1);

                file << a1.x << "," << a1.y << "," << a1.heading << "\n";
            }
        }

        if (arg == "5") {
            // std::ofstream file("C:/Users/gsaud/OneDrive/Desktop/C++/system_design/coll_1.csv");
            
            double collisionThre = 0.5;
            std::vector<agent> allAgents;

            agent a1;
            a1.x = 10;
            a1.y = 0;
            a1.speed = 0.1;     // r = v/w = 0.1/0.1 = 1. Center = (10, 1)
            a1.yawRate = 0.1;
            a1.heading = 0.0;
            allAgents.push_back(a1);

            agent a2;
            a2.x = 9;
            a2.y = 0;
            a2.speed = 0.3;        // r = v/w = 0.3/0.15 = 2. Center = (9, 2)
            a2.yawRate = 0.15;
            a2.heading = 0.0;
            allAgents.push_back(a2);

            Simulator collision;

            // fix a simulation time
            double T = 200.0;
            int total_steps = static_cast<int>(T / collision.getdt());

            for (int t = 0; t < total_steps; ++t) {
                
                for (auto& a : allAgents) {
                    collision.updatePoseExact(a);
                }
                
                // check collision
                for (int i = 0; i < allAgents.size(); ++i) {
                    for (int j = i+1; j < allAgents.size(); ++j) {
                        double dx = allAgents[i].x - allAgents[j].x;
                        double dy = allAgents[i].y - allAgents[j].y;
                        
                        if ((dx*dx + dy*dy) < collisionThre) {
                            std::cout << "Collision at step: " << t << std::endl;
                            break;
                        }
                    }
                }


                // file << a1.x << "," << a1.y << "," << a1.heading << "\n";
            }
        }
    }
}
