#include "rclcpp/rclcpp.hpp"

#include <fstream>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "util/kinematics_platform/Kinematics.hpp"

class GenerateWhiteboardAngles : public rclcpp::Node
{
public:
    GenerateWhiteboardAngles() : Node("generate_whiteboard_angles") {
        Machine machine = Machine(5.08, 7.9375, 4.44500, 9.32);
        const double startingAngle = 206.662752199;
        const double angToStep = 3200.0 / 360.0;
        int goal_pos[3] = {0, 0, 0};

        std::ofstream generated_angles;
        generated_angles.open("ski_testing3.csv");
        generated_angles << "qx, qy, qz, qw, motor0, motor1, motor2" << std::endl;
        // bool seen_val[1200][1200][1200];
        const int SIZE = 1200;
        std::vector<std::vector<std::vector<bool>>> seen_val(SIZE, std::vector<std::vector<bool>>(SIZE, std::vector<bool>(SIZE)));


        for (float or_x = -0.1f; or_x <= 0.1f; or_x += 0.001f) {
            for (float or_z = -0.1f; or_z <= 0.1f; or_z += 0.001f) {
                // for (float or_w = -1.0f; or_w <= 1.0f; or_w += 0.001f) {
                    tf2::Quaternion q(or_x, 0.f, or_z, 1.f);
                    q.normalize();
                    tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
                    tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
                    plane_normal_rot.normalize();
                    bool oob_bool = false;
                    for (int i = 0; i < 3; i++) {
                        goal_pos[i] = round((startingAngle - machine.theta(i, 10.795f, -plane_normal_rot[0], plane_normal_rot[2])) * angToStep);
                        if (goal_pos[i] < 0 || goal_pos[i] > 1200) {
                            oob_bool = true;
                        }
                    }
                    if (!oob_bool) {
                        if (!seen_val[goal_pos[0]][goal_pos[1]][goal_pos[2]]) {
                            seen_val[goal_pos[0]][goal_pos[1]][goal_pos[2]] = true;
                            generated_angles << or_x << "," << 0.f << "," << or_z << "," << 1.f << "," << goal_pos[0] << "," << goal_pos[1] << "," << goal_pos[2] << std::endl;
                        }
                    }
                // }
            }
        }
        std::cout << "done" << std::endl;
        return;
//         tf2::Quaternion q(
//             0.f,
//             0.f,
//             0.f,
//             1.f);
// int goal_pos[3] = {0, 0, 0};
//         Machine machine = Machine(2.0, 3.125, 1.75, 3.669291339);
//         const double startingAngle = 202.9;
//         const double angToStep = 3200.0 / 360.0;
//         tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
//         tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
//         bool temp_flag = false;
//         for (int i = 0; i < 3; i++) {
//             goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, -plane_normal_rot[0], plane_normal_rot[2])) * angToStep);
//         }
//         std::cout << plane_normal_rot[0] << ", " << plane_normal_rot[1] << ", " << plane_normal_rot[2] << std::endl;
//         std::cout << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << std::endl;




        // #include "util/kinematics_platform/Kinematics.hpp"

        // int goal_pos[3] = {0, 0, 0};
        // Machine machine = Machine(2.0, 3.125, 1.75, 3.669291339);
        // const double startingAngle = 202.9;
        // const double angToStep = 3200.0 / 360.0;

        // angle_data_log.open("angle_data_new_new_new.csv");
        // angle_data_log << "nx, ny, nz, motor0, motor1, motor2\n";

        // for (float or_x = -0.4; or_x <= 0.4; or_x += 0.01) {
        //     for (float or_y = -0.4; or_y <= 0.4; or_y += 0.01) {
        //         for (float or_w = -1.0; or_w <= 1.0; or_w += 0.01) {
        //             tf2::Quaternion q(
        //                 or_x,
        //                 or_y,
        //                 0.f,
        //                 or_w);

        //             tf2::Vector3 plane_normal(0.f, 1.f, 0.f);
        //             tf2::Vector3 plane_normal_rot = tf2::quatRotate(q, plane_normal);
        //             plane_normal_rot.normalize();
        //             bool temp_flag = false;
        //             for (int i = 0; i < 3; i++) {
        //                 goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, -plane_normal_rot[0], plane_normal_rot[2])) * angToStep);
                        // if (goal_pos[i] == -2147483648) {
                        //     temp_flag = true;
                        // }
        //             }
        //             if (temp_flag) {
        //                 continue;
        //             }
        //             if (plane_normal_rot[0] <= 0.001f && plane_normal_rot[1] <= 0.001f && plane_normal_rot[2] <= 0.001f) {
        //                 // continue;
        //             }
        //             angle_data_log << -plane_normal_rot[0] << ", "<< plane_normal_rot[2] << ", " << plane_normal_rot[1] << ", " << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
        //         }
        //     }
        // }

        // std::cout << "done\n";
        // angle_data_log.close(); 



        // for (float nx = -0.2; nx <= 0.2; nx+=0.001) {
            // for (int i = 0; i < 3; i++) {
            //     goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, nx, 0.0)) * angToStep);
            // }
            // angle_data_log << nx << ", "<< 0.0 << ", " << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
        // }
        // for (float ny = -0.2; ny <= 0.2; ny+=0.001) {
        //     for (int i = 0; i < 3; i++) {
        //         goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, 0.0, ny)) * angToStep);
        //     }
        //     angle_data_log << 0.0 << ", "<< ny << ", " << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
        // }
        // for (float nx = -0.2; nx <= 0.2; nx+=0.001) {
        //     for (float ny = -0.2; ny <= 0.2; ny+=0.001) {
        //         for (int i = 0; i < 3; i++) {
        //             goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, nx, ny)) * angToStep);
        //         }
        //         angle_data_log << nx << ", "<< ny << ", " << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
        //     }
        // }




// NON QUATERNION WAY
        // #include "util/kinematics_platform/Kinematics.hpp"

        // int goal_pos[3] = {0, 0, 0};
        // Machine machine = Machine(2.0, 3.125, 1.75, 3.669291339);
        // const double startingAngle = 202.9;
        // const double angToStep = 3200.0 / 360.0;
        // bool temp_flag = false;
        // angle_data_log.open("angle_data_no_quaternion.csv");
        // angle_data_log << "nx, ny, nz, motor0, motor1, motor2\n";


        // for (float nx = -0.4; nx <= 0.4; nx+=0.001) {
        //     for (float ny = -0.4; ny <= 0.4; ny+=0.001) {
        //         temp_flag = false;
        //         for (int i = 0; i < 3; i++) {
        //             goal_pos[i] = round((startingAngle - machine.theta(i, 4.25, nx, ny)) * angToStep);
        //             if (goal_pos[i] == -2147483648) {
        //                 temp_flag = true;
        //             }
        //         }
        //         if (!temp_flag) {
        //             float nz = sqrt(1 - nx*nx - ny*ny);
        //             angle_data_log << nx << ", "<< ny << ", " << nz << ", " << goal_pos[0] << ", " << goal_pos[1] << ", " << goal_pos[2] << "\n";
        //         }
        //     }
        // }

        // std::cout << "done\n";
        // angle_data_log.close();
    }

private:
    std::ofstream angle_data_log;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenerateWhiteboardAngles>());
    rclcpp::shutdown();
    return 0;
}
