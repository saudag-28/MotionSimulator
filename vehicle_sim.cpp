// 1. Program to move robot in a straight line - done
// 2. Program to move robot in a square shape - done
// 3. Program to rotate robot in place - done
// 4. Program to rotate robot in a circle - done
// 5. two agents moving, check collision
//      a. timestep based check
//      b. collision model based check


#include "include/vehicle_sim.hpp"

std::string path = "C:/Users/gsaud/OneDrive/Desktop/C++/system_design/MotionSimulator/res";

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
            std::ofstream file(path + "/circle_.csv");

            agent a1;
            a1.x = 1;
            a1.y = 0;
            a1.speed = 0.5;   // to get radius of _, r = v/w = speed/yawRate
            a1.yawRate = 0.5;
            a1.heading = 0.0;
            
            Simulator square;

            // total steps required to complete full circle
            double total_time = 2*M_PI/a1.yawRate;
            int total_steps = static_cast<int>(std::round(total_time/square.getdt()));

            for (int t = 0; t < total_steps; ++t) {
                
                // square.circleRotate(a1);
                square.updatePoseExact(a1);

                file << a1.x << "," << a1.y << "," << a1.heading << "\n";
            }
        }

        if (arg == "5") {
            
            std::ofstream file(path + "/traj_multi.csv");
            std::ofstream col_file(path + "/collision.txt");
            
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
            a2.x = 7;
            a2.y = 0;
            a2.speed = 0.4;        // r = v/w = 0.3/0.15 = 2. Center = (9, 2)
            a2.yawRate = 0.15;
            a2.heading = 0.0;
            allAgents.push_back(a2);

            Simulator collision;

            // fix a simulation time
            double T = 200.0;
            int total_steps = static_cast<int>(T / collision.getdt());

            int collision_step = -1;
            double cx, cy = 0;
            bool collision_logged = false;

            for (int t = 0; t < total_steps; ++t) {
                
                for (auto& a : allAgents) {
                    collision.updatePoseExact(a);
                }

                // log trajectory
                file << allAgents[0].x << "," << allAgents[0].y << "," << allAgents[0].heading << ","
                << allAgents[1].x << "," << allAgents[1].y << "," << allAgents[1].heading << "\n";
                
                // check collision
                if (!collision_logged) {
                    for (int i = 0; i < allAgents.size(); ++i) {
                        for (int j = i+1; j < allAgents.size(); ++j) {
                            double dx = allAgents[i].x - allAgents[j].x;
                            double dy = allAgents[i].y - allAgents[j].y;
                            
                            if (collision_step == -1 && (dx*dx + dy*dy) < collisionThre * collisionThre) {
                                std::cout << "Collision at step: " << t << std::endl;
                                collision_step = t;
                                cx = (allAgents[i].x + allAgents[j].x)/2.0;
                                cy = (allAgents[i].y + allAgents[j].y)/2.0;
                                
                                col_file << collision_step << "," << cx << "," << cy << "\n";

                                collision_logged = true;
                                break;
                            }
                        }

                        if (collision_logged) break;
                    }
                }
                if (collision_logged) break;
            }
            file.close();
            col_file.close();
        }
    }
}
