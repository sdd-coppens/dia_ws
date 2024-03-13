//
// Created by koen on 31/01/24.
//

#ifndef DIA_WS_COORDINATE_CONVERSIONS_HPP
#define DIA_WS_COORDINATE_CONVERSIONS_HPP

void robot_to_world(const std::array<fp32, 3> &robot_position,
                    std::array<fp32, 3> &world_vector)
{
    // Convert robot position to world position
    // (x, y, z) -> (-z, -x, y) / 100.0f
    world_vector[0] = -robot_position[2] / 100.0f;
    world_vector[1] = -robot_position[0] / 100.0f;
    world_vector[2] = robot_position[1] / 100.f;
}

void world_to_robot(const std::array<float, 3> &world_vector,
                    std::array<float, 3> &robot_position)
{
    // Convert world position to robot position
   // (-x, z, y) -> (-z, -x, y)
    robot_position[0] = -world_vector[2] * 100.f;
    robot_position[1] = -world_vector[0] * 100.f;
    robot_position[2] = world_vector[1] * 100.f;
}

void robot_to_world(const std::array<float, 3> &robot_position,
                    const std::array<float, 4> &robot_orientation,
                    std::array<float, 3> &world_vector,
                    std::array<float, 4> &world_orientation) {
    // Convert robot position to world position
    // (x, y, z) -> (-z, -x, y) / 100.0f
    world_vector[0] = -robot_position[2] / 100.0f;
    world_vector[1] = -robot_position[0] / 100.0f;
    world_vector[2] = robot_position[1] / 100.f;

    // Convert robot orientation to world orientation
    // (x, y, z, w) -> (-x, z, y, w)
    world_orientation[0] = -robot_orientation[0];
    world_orientation[1] = robot_orientation[2];
    world_orientation[2] = robot_orientation[1];
    world_orientation[3] = robot_orientation[3];
}


//TODO: scalings?
void world_to_robot(const std::array<float, 3> &world_vector,
                    const std::array<float, 4> &world_orientation,
                    std::array<float, 3> &robot_position,
                    std::array<float, 4> &robot_orientation) {
    // Convert world position to robot position
    // (-x, z, y) -> (-z, -x, y)
    robot_position[0] = -world_vector[2] * 100.f;
    robot_position[1] = -world_vector[0] * 100.f;
    robot_position[2] = world_vector[1] * 100.f;

    // Convert world orientation to robot orientation
    // (-x, z, y, w) -> (x, y, z, w) * 100
    robot_orientation[0] = -world_orientation[0] * 100;
    robot_orientation[1] = world_orientation[2] * 100;
    robot_orientation[2] = world_orientation[1] * 100;
    robot_orientation[3] = world_orientation[3] * 100;
}


#endif //DIA_WS_COORDINATE_CONVERSIONS_HPP
