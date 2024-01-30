//
// Created by koen on 29/01/24.
//

#ifndef DIA_WS_PERTURBMOTION_HPP
#define DIA_WS_PERTURBMOTION_HPP


#include <eigen3/Eigen/Dense>


void barycentric_coordinates(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,
                             Eigen::Vector3f &barycentric_coordinates);

void project_point_onto_plane(const Eigen::Vector3f &p, const Eigen::Vector3f &normal, const Eigen::Vector3f &a,
                              Eigen::Vector3f &projected_point);

void closest_point_on_triangle(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,
                               Eigen::Vector3f &closest_point);

void closest_face_on_mesh(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                          Eigen::Vector3f &closest_point);

float f(float x);


void perturb_motion(const Eigen::Vector3f &p,
                    const Eigen::Vector3f &closest_point,
                    const Eigen::Vector3f &closest_point_remote,
                    Eigen::Vector3f &perturbed_point);

void get_new_point(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                   const Eigen::Vector3f &translation_operator,
                   const Eigen::Matrix<float, 3, 3> &rotation_operator,
                   const Eigen::Vector3f &translation_remote,
                   const Eigen::Matrix<float, 3, 3> &rotation_remote,
                   Eigen::Vector3f &perturbed_point);


void remote_point(const Eigen::Vector3f &p,
                  const Eigen::Vector3f &translation_operator,
                  const Eigen::Matrix<float, 3, 3> &rotation_operator,
                  const Eigen::Vector3f &translation_remote,
                  const Eigen::Matrix<float, 3, 3> &rotation_remote,
                  Eigen::Vector3f &p_remote);

float point_to_triangle_distance(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle);


#endif //DIA_WS_PERTURBMOTION_HPP
