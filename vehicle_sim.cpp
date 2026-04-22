// 1. Program to move robot in a straight line - done
// 2. Program to move robot in a square shape - done
// 3. Program to rotate robot in place - done
// 4. Program to rotate robot in a circle - done
// 5. two agents moving, check collision
//      a. timestep based check
//      b. collision model based check


#include "vehicle_sim.hpp"


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
