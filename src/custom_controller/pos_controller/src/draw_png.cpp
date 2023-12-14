#include "rclcpp/rclcpp.hpp"
#include "xarm/wrapper/xarm_api.h"

#include "lodepng.h"
#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <memory>
#include <fstream>

struct Pixel {
    int x, y;
    Pixel(int x, int y) : x(x), y(y) {}
};

double calculateDistance(const Pixel& p1, const Pixel& p2) {
    int dx = p1.x - p2.x;
    int dy = p1.y - p2.y;
    return std::sqrt(dx * dx + dy * dy);
}
struct Point {
    double x, y, z;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("draw_png");

    std::string port = "192.168.1.171";

    XArmAPI *arm = new XArmAPI(port);
    sleep_milliseconds(500);
    if (arm->error_code != 0)
        arm->clean_error();
    if (arm->warn_code != 0)
        arm->clean_warn();
    arm->motion_enable(true);
    arm->set_mode(0);
    arm->set_state(0);
    sleep_milliseconds(500);

    // arm->reset(true);
    fp32 first_pose[6] = {110, 0, 220, 180, 0, 0};
    arm->set_position(first_pose, true);
    arm->set_mode(1);
    arm->set_state(0);
    sleep_milliseconds(100);

    arm->set_reduced_max_joint_speed(100);
    arm->set_reduced_mode(true);


    std::ifstream inputFile("coordinates.txt");

    if (!inputFile.is_open()) {
        std::cerr << "Error opening file." << std::endl;
        return 1;
    }

    std::string line;
    while (std::getline(inputFile, line)) {
        std::regex regexPattern("x:\\s*([\\d.]+),\\s*y:\\s*([\\d.]+),\\s*z:\\s*([\\d.]+)");
        std::smatch match;

        if (std::regex_match(line, match, regexPattern)) {
            Point point;
            point.x = std::stod(match[1]);
            point.y = std::stod(match[2]);
            point.z = std::stod(match[3]);

            // Process the parsed point (e.g., store it in a data structure or perform some operation)
            // std::cout << "Parsed point: x=" << point.x << ", y=" << point.y << ", z=" << point.z << std::endl;
            if (point.z == 215) {
                continue;
            } else if (point.z == 220) {
                fp32 pose[6] = {point.x / 3 + 100, point.y / 3, 220, 180, 0, 0};
                arm->set_servo_cartesian(pose, 1);
                sleep_milliseconds(4);
            }
        } else {
            std::cerr << "Error parsing line: " << line << std::endl;
        }
    }
    inputFile.close();

    // int count = 0;
    // // Print the coordinates of the continuous path
    // for (const auto& coordinates : finalReducedPath) {

    //     count++;
    //     std::cout << "(" << coordinates.x << "," << coordinates.y << ") ";
    //     log_file << "x:"<< coordinates.x << ", y:" <<  coordinates.y << ", z:" << 220 << "\n";
        // fp32 pose[6] = {coordinates.x / 3 + 100, coordinates.y / 3, 220, 180, 0, 0};
        // arm->set_servo_cartesian(pose, 1);
        // sleep_milliseconds(50);
    // }

// colcon build --packages-select pos_controller && source install/setup.bash
    rclcpp::spin_some(node);
    rclcpp::shutdown();
}

// #include "rclcpp/rclcpp.hpp"
// #include "xarm/wrapper/xarm_api.h"

// #include "lodepng.h"
// #include <iostream>
// #include <vector>
// #include <queue>
// #include <cmath>
// #include <memory>
// #include <fstream>

// struct Pixel {
//     int x, y;
//     Pixel(int x, int y) : x(x), y(y) {}
// };

// double calculateDistance(const Pixel& p1, const Pixel& p2) {
//     int dx = p1.x - p2.x;
//     int dy = p1.y - p2.y;
//     return std::sqrt(dx * dx + dy * dy);
// }

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("draw_png");

//     std::string port = "192.168.1.171";

//     XArmAPI *arm = new XArmAPI(port);
//     sleep_milliseconds(500);
//     if (arm->error_code != 0)
//         arm->clean_error();
//     if (arm->warn_code != 0)
//         arm->clean_warn();
//     arm->motion_enable(true);
//     arm->set_mode(0);
//     arm->set_state(0);
//     sleep_milliseconds(500);

//     // arm->reset(true);
//     fp32 first_pose[6] = {110, 0, 220, 180, 0, 0};
//     arm->set_position(first_pose, true);
//     arm->set_mode(1);
//     arm->set_state(0);
//     sleep_milliseconds(100);

//     arm->set_reduced_max_joint_speed(100);
//     arm->set_reduced_mode(true);


//     std::ofstream log_file;
//     log_file.open("coordinates.txt");

//     const char* filename = "/home/stijn/thesis/xarm_ws/src/custom_controller/pos_controller/src/frog.png";  // Replace with your PNG file path

//     std::vector<unsigned char> image; // The raw pixels will be stored here
//     unsigned width, height;

//     // Decode the image
//     unsigned error = lodepng::decode(image, width, height, filename);

//     // Check if the image decoding was successful
//     if (error) {
//         std::cerr << "Error decoding image: " << lodepng_error_text(error) << std::endl;
//         return 1;
//     }

//     // Assuming each pixel has RGBA channels, so 4 channels per pixel
//     const int channels = 4;

//     // Create a 2D array to store visited pixels
//     std::vector<std::vector<bool>> visited(height, std::vector<bool>(width, false));

//     // Create a vector to store the further reduced path
//     std::vector<Pixel> furtherReducedPath;

//     // Function to check if a pixel is black
//     auto isBlack = [&](int x, int y) {
//         unsigned index = y * width * channels + x * channels;
//         return (image[index] == 0 && image[index + 1] == 0 && image[index + 2] == 0);
//     };

//     // Function to check if a pixel is within the image bounds
//     auto isValid = [&](int x, int y) {
//         return (x >= 0 && x < width && y >= 0 && y < height);
//     };

//     // Define possible neighbor directions (up, down, left, right)
//     const int dx[] = {0, 0, -1, 1};
//     const int dy[] = {-1, 1, 0, 0};

//     // Find a starting black pixel
//     int startX = -1, startY = -1;
//     for (unsigned y = 0; y < height; ++y) {
//         for (unsigned x = 0; x < width; ++x) {
//             if (isBlack(x, y)) {
//                 startX = x;
//                 startY = y;
//                 break;
//             }
//         }
//         if (startX != -1) break;
//     }

//     // If no black pixel is found, exit
//     if (startX == -1) {
//         std::cerr << "No black pixel found in the image." << std::endl;
//         return 1;
//     }

//     // Initialize a queue for BFS traversal
//     std::queue<Pixel> bfsQueue;

//     // Start BFS from the first black pixel
//     bfsQueue.push(Pixel(startX, startY));
//     visited[startY][startX] = true;

//     // Traverse the further reduced path using BFS
//     while (!bfsQueue.empty()) {
//         Pixel currentPixel = bfsQueue.front();
//         bfsQueue.pop();

//         furtherReducedPath.push_back(currentPixel);

//         // Check neighbors
//         for (int i = 0; i < 4; ++i) {
//             int nx = currentPixel.x + dx[i];
//             int ny = currentPixel.y + dy[i];

//             // Check if the neighbor is a valid black pixel
//             if (isValid(nx, ny) && isBlack(nx, ny) && !visited[ny][nx]) {
//                 bfsQueue.push(Pixel(nx, ny));
//                 visited[ny][nx] = true;
//             }
//         }
//     }

//     // Further reduce the path by skipping consecutive close pixels
//     const double distanceThreshold = 1.0;
//     std::vector<Pixel> finalReducedPath;

//     for (size_t i = 0; i < furtherReducedPath.size(); ++i) {
//         if (i == 0 || i == furtherReducedPath.size() - 1 ||
//             calculateDistance(furtherReducedPath[i - 1], furtherReducedPath[i]) > distanceThreshold) {
//             finalReducedPath.push_back(furtherReducedPath[i]);
//         }
//     }

//     int count = 0;
//     // Print the coordinates of the continuous path
//     for (const auto& coordinates : finalReducedPath) {

//         count++;
//         std::cout << "(" << coordinates.x << "," << coordinates.y << ") ";
//         log_file << "x:"<< coordinates.x << ", y:" <<  coordinates.y << ", z:" << 220 << "\n";
//         // fp32 pose[6] = {coordinates.x / 3 + 100, coordinates.y / 3, 220, 180, 0, 0};
//         // arm->set_servo_cartesian(pose, 1);
//         // sleep_milliseconds(50);
//     }

// // colcon build --packages-select pos_controller && source install/setup.bash
//     std::cout << count << std::endl;
//     rclcpp::spin_some(node);
//     rclcpp::shutdown();
// }