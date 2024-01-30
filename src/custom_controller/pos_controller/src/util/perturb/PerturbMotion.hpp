//
// Created by koen on 29/01/24.
//
#ifndef DIA_WS_PERTURBMOTION_HPP2
#define DIA_WS_PERTURBMOTION_HPP2

#include <eigen3/Eigen/Dense>

void barycentric_coordinates(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,
                             Eigen::Vector3f &barycentric_coordinates) {
    Eigen::Vector3f v0 = triangle.col(0) - triangle.col(2);
    Eigen::Vector3f v1 = triangle.col(1) - triangle.col(2);
    Eigen::Vector3f v2 = p - triangle.col(2);

    float d00 = v0.dot(v0);
    float d01 = v0.dot(v1);
    float d11 = v1.dot(v1);
    float d20 = v2.dot(v0);
    float d21 = v2.dot(v1);

    float denominator = d00 * d11 - d01 * d01;

    barycentric_coordinates.x() = (d11 * d20 - d01 * d21) / denominator;
    barycentric_coordinates.y() = (d00 * d21 - d01 * d20) / denominator;
    barycentric_coordinates.z() = 1.0f - barycentric_coordinates.x() - barycentric_coordinates.y();
}

void project_point_onto_plane(const Eigen::Vector3f &p, const Eigen::Vector3f &normal, const Eigen::Vector3f &a,
                              Eigen::Vector3f &projected_point) {
    Eigen::Vector3f v = p - a;
    float d = v.dot(normal);
    projected_point = p - d * normal;
}

void closest_point_on_triangle(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,
                               Eigen::Vector3f &closest_point) {
    Eigen::Vector3f a = triangle.col(0);
    Eigen::Vector3f b = triangle.col(1);
    Eigen::Vector3f c = triangle.col(2);

    Eigen::Vector3f plane_normal = (b - a).cross(c - a);
    plane_normal.normalize();

    Eigen::Vector3f projected_point;
    project_point_onto_plane(p, plane_normal, a, projected_point);

    Eigen::Vector3f barycentric_coords;
    barycentric_coordinates(projected_point, triangle, barycentric_coords);

    //    clip coordinates to triangle
    if (barycentric_coords.x() < 0.0f) {
        float t = (p - b).dot(c - b) / (c - b).dot(c - b);
        t = std::max(0.0f, std::min(1.0f, t));
        closest_point.x() = 0.0f;
        closest_point.y() = 1.0f - t;
        closest_point.z() = t;
    } else if (barycentric_coords.y() < 0.0f) {
        float t = (p - c).dot(a - c) / (a - c).dot(a - c);
        t = std::max(0.0f, std::min(1.0f, t));
        closest_point.x() = t;
        closest_point.y() = 0.0f;
        closest_point.z() = 1.0f - t;
    } else if (barycentric_coords.z() < 0.0f) {
        float t = (p - a).dot(b - a) / (b - a).dot(b - a);
        t = std::max(0.0f, std::min(1.0f, t));
        closest_point.x() = 1.0f - t;
        closest_point.y() = t;
        closest_point.z() = 0.0f;
    } else {
        closest_point = barycentric_coords;
    }
}

float f(float x) {
    if (x < 0.0f) {
        return 0.0f;
    } else if (x > 1.0f) {
        return 1.0f;
    } else {
        return -2.0f * x * x * x + 3.0f * x * x;
    }
}

float point_to_triangle_distance(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle) {
    Eigen::Vector3f closest_point;
    closest_point_on_triangle(p, triangle, closest_point);
    return (p - closest_point).norm();
}

void closest_face_on_mesh(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                          Eigen::Vector3f &closest_point) {
    //TODO: replace with AABB tree or kd-tree
    float min_distance = std::numeric_limits<float>::max();
    for (const auto &triangle: triangles) {
        Eigen::Vector3f closest_point_triangle;
        closest_point_on_triangle(p, triangle, closest_point_triangle);
        //to cartesian
        closest_point_triangle =
                closest_point_triangle.x() * triangle.col(0) + closest_point_triangle.y() * triangle.col(1) +
                closest_point_triangle.z() * triangle.col(2);

        float distance = (p - closest_point_triangle).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = closest_point_triangle;
        }
    }
}

void remote_point(const Eigen::Vector3f &p, const Eigen::Vector3f &translation_operator,
                  const Eigen::Matrix<float, 3, 3> &rotation_operator, const Eigen::Vector3f &translation_remote,
                  const Eigen::Matrix<float, 3, 3> &rotation_remote, Eigen::Vector3f &p_remote) {
    p_remote = rotation_remote * (rotation_operator.transpose() * (p - translation_operator)) + translation_remote;
}

void perturb_motion(const Eigen::Vector3f &p, const Eigen::Vector3f &closest_point,
                    const Eigen::Vector3f &closest_point_remote, Eigen::Vector3f &perturbed_point) {
    Eigen::Vector3f v = closest_point_remote - closest_point;
    Eigen::Vector3f d = p - closest_point;

    float ratio = 1 - d.norm() / (v.norm() * 3.0f / 2.0f);

    perturbed_point = p + f(ratio) * v;
}

void get_new_point(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                   const Eigen::Vector3f &translation_operator,
                   const Eigen::Matrix<float, 3, 3> &rotation_operator,
                   const Eigen::Vector3f &translation_remote,
                   const Eigen::Matrix<float, 3, 3> &rotation_remote,
                   Eigen::Vector3f &perturbed_point) {

    Eigen::Vector3f closest_point;
    closest_face_on_mesh(p, triangles, closest_point);

    Eigen::Vector3f closest_point_remote;
    remote_point(closest_point, translation_operator, rotation_operator, translation_remote, rotation_remote,
                 closest_point_remote);

    perturb_motion(p, closest_point, closest_point_remote, perturbed_point);
}

#endif