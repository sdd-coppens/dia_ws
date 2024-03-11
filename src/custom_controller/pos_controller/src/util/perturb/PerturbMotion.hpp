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

void inline project_point_onto_plane(const Eigen::Vector3f &p, const Eigen::Vector3f &normal, const Eigen::Vector3f &a,
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

    //back to world coordinates
    closest_point = closest_point.x() * triangle.col(0) + closest_point.y() * triangle.col(1) +
                    closest_point.z() * triangle.col(2);
}

float inline f(float x) {
    if (x < 0.0f) {
        return 0.0f;
    } else if (x > 1.0f) {
        return 1.0f;
    } else {
//        return x;
        return -2.0f * x * x * x + 3.0f * x * x;
    }
}

float inline point_to_triangle_distance(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle) {
    Eigen::Vector3f closest_point;
    closest_point_on_triangle(p, triangle, closest_point);

    return (p - closest_point).norm();
}

void closest_point_on_mesh(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                          Eigen::Vector3f &closest_point) {
    //TODO: replace with AABB tree or kd-tree
    float min_distance = std::numeric_limits<float>::max();

    Eigen::Vector3f ray_origin = p;
    Eigen::Vector3f ray_direction = Eigen::Vector3f(1.0f, 0.0f, 0.0f);
    int intersections = 0;

    std::vector<Eigen::Vector3f> intersection_points;

    for (const auto &triangle: triangles) {

//        printf ("triangle: \n");
//        printf (" a: %f, %f, %f\n", triangle.col(0).x(), triangle.col(0).y(), triangle.col(0).z());
//        printf (" b: %f, %f, %f\n", triangle.col(1).x(), triangle.col(1).y(), triangle.col(1).z());
//        printf (" c: %f, %f, %f\n", triangle.col(2).x(), triangle.col(2).y(), triangle.col(2).z());
//
        Eigen::Vector3f closest_point_triangle;
        closest_point_on_triangle(p, triangle, closest_point_triangle);

        float distance = (p - closest_point_triangle).norm();
        if (distance < min_distance) {
            min_distance = distance;
            closest_point = closest_point_triangle;
        }

        Eigen::Vector3f a = triangle.col(0);
        Eigen::Vector3f b = triangle.col(1);
        Eigen::Vector3f c = triangle.col(2);

        Eigen::Vector3f plane_normal = (b - a).cross(c - a);
        plane_normal.normalize();

        //intersection distance
        if (plane_normal.dot(ray_direction) == 0.0f) {
            continue; //ray is parallel to plane
        }

        else {
            // distance to plane
            float t = plane_normal.dot(a - ray_origin) / plane_normal.dot(ray_direction);


//              printf("t: %f\n", t);
            if (t < 0.0f) {
                continue; // intersection point is behind ray origin // we only have to look at one direction
            }


            // intersection point
            Eigen::Vector3f intersection_point = ray_origin + t * ray_direction;

//            printf("intersection point: %f, %f, %f\n", intersection_point.x(), intersection_point.y(), intersection_point.z());

            // check if intersection point is inside triangle
            // move triangle so that intersection point is at origin
            Eigen::Matrix<float, 3, 3> triangle_moved;
            triangle_moved.col(0) = a - intersection_point;
            triangle_moved.col(1) = b - intersection_point;
            triangle_moved.col(2) = c - intersection_point;

            //vertex normals of the triangle
            Eigen::Vector3f n0 = triangle_moved.col(1).cross(triangle_moved.col(2));
            Eigen::Vector3f n1 = triangle_moved.col(2).cross(triangle_moved.col(0));
            Eigen::Vector3f n2 = triangle_moved.col(0).cross(triangle_moved.col(1));

            //check if vertex normals are facing in the same direction
            if (n0.dot(n1)  < 0.0f || n0.dot(n2) < 0.0f) {
//                printf("intersection point is outside triangle\n");
                continue;
            }

            // check if intersection point is unqiue, because one point can be on multiple two triangles... (edges and vertices)
            bool is_unique = true;
            for (size_t i = 0; i < intersection_points.size(); i++) {
                if ((intersection_points[i] - intersection_point).norm() < 0.0001f) {
                    is_unique = false;
                    break;
                }
            }

            if (is_unique) {
                intersection_points.push_back(intersection_point);
                intersections++;
            }
        }
    }

//    printf("intersections: %d\n", intersections);
    if (intersections % 2 == 1){
        closest_point = p;
    }

}

void inline remote_point(const Eigen::Vector3f &p, const Eigen::Vector3f &translation_operator,
                  const Eigen::Matrix<float, 3, 3> &rotation_operator, const Eigen::Vector3f &translation_remote,
                  const Eigen::Matrix<float, 3, 3> &rotation_remote, Eigen::Vector3f &p_remote) {
    p_remote = rotation_remote * (rotation_operator.transpose() * (p - translation_operator)) + translation_remote;
}

void perturb_motion(const Eigen::Vector3f &p, const Eigen::Vector3f &closest_point,
                    const Eigen::Vector3f &closest_point_remote, Eigen::Vector3f &perturbed_point) {
    Eigen::Vector3f p_delta = closest_point_remote - closest_point;
    Eigen::Vector3f p_min = p - closest_point;

//    float ratio = 1 - p_min.norm() / ( p_delta.norm() * (3.0f / 2.0f)); //TODO: take better look at this. It should not be 1 whenever p_delta is smaller.
    // it should be 1, when p_min.norm() is zero, and should be 1 when p_min.norm() is equal to p_delta.norm() * (3.0f / 2.0f)


//    float ratio = p_min.norm() / (p_delta.norm() * (3.0f / 2.0f));

    float ratio;

    if (p_min.norm() < 0.02) { //tiny "deadzone" to avoid jitter
        ratio = 1.0;
    }
    else if ( (p_min.norm() - 0.02) < (p_delta.norm() * (3.0f / 2.0f))) {
        ratio = 1 - (p_min.norm() - 0.02) / (p_delta.norm() * (3.0f / 2.0f));
    } else {
        ratio = 0.0f;
    }

    perturbed_point = p + f(ratio) * p_delta;
}

void get_new_point(const Eigen::Vector3f &p, const float point_radius,
                   const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                   const Eigen::Vector3f &translation_operator,
                   const Eigen::Matrix<float, 3, 3> &rotation_operator,
                   const Eigen::Vector3f &translation_remote,
                   const Eigen::Matrix<float, 3, 3> &rotation_remote,
                   Eigen::Vector3f &perturbed_point) {

    Eigen::Vector3f closest_point;
    closest_point_on_mesh(p, triangles, closest_point);

    Eigen::Vector3f closest_point_remote;
    remote_point(closest_point, translation_operator, rotation_operator, translation_remote, rotation_remote,
                 closest_point_remote);

    Eigen::Vector3f closest_point_on_pointer = p - point_radius * (p - closest_point).normalized();

    perturb_motion(closest_point_on_pointer, closest_point, closest_point_remote, perturbed_point);
}

#endif