//
// Created by koen on 06/03/24.
// Unit tests for the perturb motion functions
//

#include <vector>

#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include <catch2/catch.hpp>

//#include "util/perturb/PerturbMotion.hpp"
#include "../src/util/perturb/PerturbMotion.hpp"

/**
Test cases for:

void barycentric_coordinates(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,
                             Eigen::Vector3f &barycentric_coordinates)

void project_point_onto_plane(const Eigen::Vector3f &p, const Eigen::Vector3f &normal, const Eigen::Vector3f &a,
                              Eigen::Vector3f &projected_point) {


void closest_point_on_triangle(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle,


float f(float x)

float point_to_triangle_distance(const Eigen::Vector3f &p, const Eigen::Matrix<float, 3, 3> &triangle)

void closest_face_on_mesh(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                          Eigen::Vector3f &closest_point) {


void remote_point(const Eigen::Vector3f &p, const Eigen::Vector3f &translation_operator,
                  const Eigen::Matrix<float, 3, 3> &rotation_operator, const Eigen::Vector3f &translation_remote,

void perturb_motion(const Eigen::Vector3f &p, const Eigen::Vector3f &closest_point,
                    const Eigen::Vector3f &closest_point_remote, Eigen::Vector3f &perturbed_point)

void get_new_point(const Eigen::Vector3f &p, const std::vector<Eigen::Matrix<float, 3, 3>> &triangles,
                   const Eigen::Vector3f &translation_operator,
                   const Eigen::Matrix<float, 3, 3> &rotation_operator,
                   const Eigen::Vector3f &translation_remote,
                   const Eigen::Matrix<float, 3, 3> &rotation_remote,
                   Eigen::Vector3f &perturbed_point(
*/

//Barycentric test case

TEST_CASE("Barycentric coordinates test case") {
    Eigen::Matrix3f triangle;

    //TODO: test with triangle that has two or more points at the same point.
    //TODO: test with wrong sizes

    triangle << 0.0f, 0.0f, 0.0f,
            1.0f, 0.0f, 0.0f,
            0.0f, 1.0f, 0.0f;

    /*                  * (0,1,0
     *                  |   \
     *                  |      \
     *                  |          \
     * - - - - - - - - -*------------* - - - -
     *                  |(0,0,0)      (0,0,1)
     *                  |
     *                  |
     *                  |
     *                  |
     */

    Eigen::Vector3f barycentric_coords;



    SECTION("middle triangle point"){
        Eigen::Vector3f p(0.0f, 1.0/3.0f, 1.0/3.0f);
        barycentric_coordinates(p, triangle, barycentric_coords);
        REQUIRE_THAT(barycentric_coords.x(), Catch::Matchers::WithinAbs(1.f/3.f, 0.0001));
        REQUIRE_THAT(barycentric_coords.y(), Catch::Matchers::WithinAbs(1.f/3.f, 0.0001));
        REQUIRE_THAT(barycentric_coords.z(), Catch::Matchers::WithinAbs(1.f/3.f, 0.0001));

        REQUIRE_THAT(barycentric_coords.sum(), Catch::Matchers::WithinAbs(1.0f, 0.0001));
    }

    SECTION("Outside triangle point"){
        Eigen::Vector3f p(-1.0f, 0.2f, -0.2f);


/*                  * (0,1,0
 *                  |   \
 *                  |      \
 *  (-1,0.2,-0.2) * |         \
 * - - - - - - - - -*------------* - - - -
 *                  |(0,0,0)      (0,0,1)
 *                  |
 *                  |
 *                  |
 *                  |
 */
        barycentric_coordinates(p, triangle, barycentric_coords);

        Eigen::Vector3f p_reversed = barycentric_coords.x() * triangle.col(0) + barycentric_coords.y() * triangle.col(1) + barycentric_coords.z() * triangle.col(2);

        REQUIRE(p_reversed == Eigen::Vector3f(0.0f, 0.2f, -0.2f));
    }

    SECTION("On triangle point"){
        Eigen::Vector3f p(0.0f, 0.0f, 0.0f);
        barycentric_coordinates(p, triangle, barycentric_coords);
        REQUIRE(barycentric_coords.x() == 0.0f);
        REQUIRE(barycentric_coords.y() == 0.0f);
        REQUIRE(barycentric_coords.z() == 1.0f);
    }

    SECTION("On triangle edge"){
        Eigen::Vector3f p(0.0f, 0.5f, 0.0f);
        barycentric_coordinates(p, triangle, barycentric_coords);
        REQUIRE(barycentric_coords.x() == 0.5f);
        REQUIRE(barycentric_coords.y() == 0.0f);
        REQUIRE(barycentric_coords.z() == 0.5f);


        barycentric_coordinates(Eigen::Vector3f(0.0f, 0.0f, 0.5f), triangle, barycentric_coords);
        REQUIRE(barycentric_coords.x() == 0.0f);
        REQUIRE(barycentric_coords.y() == 0.5f);
        REQUIRE(barycentric_coords.z() == 0.5f);

        barycentric_coordinates(Eigen::Vector3f(0.0f, 0.0f, 0.2f), triangle, barycentric_coords);
        REQUIRE(barycentric_coords.x() == 0.0f);
        REQUIRE(barycentric_coords.y() == 0.2f);
        REQUIRE(barycentric_coords.z() == 0.8f);

    }

    SECTION("Benchmarks"){
        BENCHMARK("barycentric bench"){
            return barycentric_coordinates(Eigen::Vector3f(0.5f, 0.5f, 0.0f), triangle, barycentric_coords);
        };
    }
}

TEST_CASE("Barycentric Clipping"){
    //TODO:: add triangle that is not on an axis plane
    Eigen::Matrix3f triangle;

    triangle << 0.0f, 0.0f, 0.0f,
    1.0f, 0.0f, 0.0f,
    0.0f, 1.0f, 0.0f;


    SECTION("inside"){
        Eigen::Vector3f p(0.0, 0.1, 0.1);
        Eigen::Vector3f closest_point;
        closest_point_on_triangle(p, triangle, closest_point);

        REQUIRE(p == closest_point);

    }

    SECTION("edge"){
        Eigen::Vector3f p(0.0, 0.3, 0.0);
        Eigen::Vector3f closest_point;
        closest_point_on_triangle(p, triangle, closest_point);

        REQUIRE(p == closest_point);
    }

    SECTION("outside - X"){
        /*                  * (0,1,0
         *                  |   \
         *                  |      \
         *                  |          \
         * - - - - - - - - -*------------* - - - -
         *                  |(0,0,0)      (0,0,1)
         *                  |(-1,0,0) P -> STACKED in depth
         *                  |
         *                  |
         *                  |
         */

        Eigen::Vector3f p(-1.0, 0.0, 0.0);
        Eigen::Vector3f closest_point;

        closest_point_on_triangle(p, triangle, closest_point);

        REQUIRE(closest_point == Eigen::Vector3f(0.0, 0.0, 0.0));
    }

    SECTION("outside - Y"){
        /*                  * (0,1,0
         *                  |   \
         *                  |      \
         *                  |          \
         * - - - - - - - - -*------------* - - - -
         *                  |(0,0,0)      (0,0,1)
         *                  |
         *                  |
         *                  *(0,-1,0) p
         *                  |
         *                  |
         *                  |
         */

        Eigen::Vector3f p(0.0, -1.0, 0.0);
        Eigen::Vector3f closest_point;

        closest_point_on_triangle(p, triangle, closest_point);

        REQUIRE(closest_point == Eigen::Vector3f(0.0, 0.0, 0.0));
    }

    SECTION("outside - Z"){
        /*                  * (0,1,0
         *                  |   \
         *                  |      \
         *                  |          \
         * - - - - - - - - -*------------* - - - - *(0,0,2) p
         *                  |(0,0,0)      (0,0,1)
         *                  |
         *                  |
         *                  |
         *                  |
         *                  |
         *                  |
         */

        Eigen::Vector3f p(0.0, -0.0, 2.0);
        Eigen::Vector3f closest_point;

        closest_point_on_triangle(p, triangle, closest_point);

        REQUIRE(closest_point == Eigen::Vector3f(0.0, 0.0, 1.0));
        }

}

TEST_CASE("f - smoothing function"){

    //test < 0 case
    REQUIRE(f(-1.0f) == 0.0f);

    //test 0 case
    REQUIRE(f(0.0f) == 0.0f);

    //test 0 < x < 1 case
    float result = f(0.5);
    REQUIRE( (result <= 1.0 && result >= 0.0f) );

    std::vector<float> inputs;
    std::vector<float> outputs;

    inputs.reserve(100);
    outputs.reserve(100);

    for( int i = 0; i < 100; i++){
        inputs.push_back(i * 1.0/100);
        outputs.push_back(f(inputs[i]));
    }

    for (size_t i = 0; i < inputs.size(); i++){
        REQUIRE(outputs[i] <= 1.0f);
        REQUIRE(outputs[i] >= 0.0f);
        if (i > 0){
            REQUIRE(outputs[i] >= outputs[i-1]);
        }
    }

    //test 1 case
    REQUIRE(f(1.0f) == 1.0f);

    //test > 1 case
    REQUIRE(f(2.0f) == 1.0f);
}

TEST_CASE("Point to triangle distance"){

    SECTION("triangle1"){
        Eigen::Matrix3f triangle;

        triangle << 0.0f, 0.0f, 0.0f,
                1.0f, 0.0f, 0.0f,
                0.0f, 1.0f, 0.0f;

        Eigen::Vector3f p(0.0, 0.0, 0.0);
        REQUIRE(point_to_triangle_distance(p, triangle) == 0.0f);

        p = Eigen::Vector3f(0.0, 0.0, 1.0);
        REQUIRE(point_to_triangle_distance(p, triangle) == 0.0f);

        p = Eigen::Vector3f(0.0, 0.0, -1.0);
        REQUIRE(point_to_triangle_distance(p, triangle) == 1.0f);

        p = Eigen::Vector3f(0.0, 0.5, 0.0);
        REQUIRE(point_to_triangle_distance(p, triangle) == 0.0f);

        p = Eigen::Vector3f(0.5, 0.5, 0.0);
        REQUIRE(point_to_triangle_distance(p, triangle) == 0.5f);

        p = Eigen::Vector3f(0.5, 0.5, 1.0);
        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(0.617 , 0.01));

        p = Eigen::Vector3f(0.5, 0.5, -1.0);
        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(sqrt(1 + 0.5 * 0.5), 0.0001));

        p = Eigen::Vector3f(0.5, 0.5, 0.5);
        REQUIRE(point_to_triangle_distance(p, triangle) == 0.5f);

        p = Eigen::Vector3f(0.5, 0.5, -0.5);
        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(sqrt(0.5 * 0.5 + 0.5 * 0.5), 0.0001));

        p = Eigen::Vector3f(-1.0, 0.5, 0.5);
        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(1.0, 0.0001));

    }

    SECTION("triangle2"){
        Eigen::Matrix3f triangle;

        triangle << 0.0f, 2.f/3.0f, 0.0f,
                1.0f, 0.0f, 0.5f,
                0.0f, 1.0f, 1.0f;

        Eigen::Vector3f p(2.0/3.0, 0.0, 1.0);
        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(0.0, 0.0001));

//        p = Eigen::Vector3f(0.0, 0.0, 0.0);
//        REQUIRE_THAT(point_to_triangle_distance(p, triangle), Catch::Matchers::WithinAbs(???, 0.0001));
    }

    SECTION("benchmark closest point to triangle"){

        Eigen::Matrix3f triangle;

        triangle << 0.0f, 0.0f, 0.0f,
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f;

        Eigen::Vector3f p(0.1, 0.1, 0.0);
        BENCHMARK("closest point to triangle"){

            return point_to_triangle_distance(p, triangle);
        };
    }


}

TEST_CASE("Closest point on mesh"){

    std::vector<Eigen::Matrix3f> triangles;
    /* rectangle points
          *
              7--------6
             /|       /|
            / |      / |
           4--------5  |
           |  |     |  |
           |  3-----|--2
           | /      | /
           |/       |/
           0--------1

          coordinate system
           y    z
           |  /
           | /
           |/
           o------> x

           a point (x, y, z) is (right, up, backward)

          */

    // Initialize cube_points with an initializer list
    Eigen::Vector3f cube_points[] = {

            {-0.935f, -0.035f, -0.705f}, {0.935f, -0.035f, -0.705f}, {0.935f, -0.035f, 0.705f },
            {-0.935f, -0.035f, 0.705f }, {-0.935f, 0.035f, -0.705f }, {0.935f, 0.035f, -0.705f },
            {0.935f, 0.035f, 0.705f }, {-0.935f, 0.035f, 0.705f }
    };

    // Define a mapping from points to triangles
    int triangle_indices[][3] = {
            {0, 1, 2}, {0, 2, 3}, {0, 1, 4}, {1, 4, 5},
            {0, 3, 4}, {3, 4, 7}, {1, 2, 5}, {2, 5, 6},
            {2, 3, 6}, {3, 6, 7}, {4, 5, 6}, {4, 6, 7}
    };

    for (int i = 0; i < 12; ++i) {
        Eigen::Matrix3f triangle;
        for (int j = 0; j < 3; ++j) {
            triangle.col(j) = cube_points[triangle_indices[i][j]];
        }
        triangles.push_back(triangle);
    }


    std::vector<Eigen::Matrix3f> triangles_tetrahedron;
    /* tetrahedron points
          *
              3
             /|\
            / | \
           /  |  \
          /   |   \
         0----|----1
              2

          coordinate system
           y    z
           |  /
           | /s
           |/
           o------> x

           a point (x, y, z) is (right, up, backward)

          */

    Eigen::Vector3f tetrahedron_points[] = {
            {1/3 * sqrt(3), 0, 0}, {-1/6 * sqrt(3), 0.5, 0}, {-1/6 * sqrt(3), -0.5, 0}, {0, 0, 1/3 * sqrt(6)}
    };

    int tetrahedron_triangle_indices[][3] = {
            {0, 1, 2}, {0, 1, 3}, {0, 2, 3}, {1, 2, 3}
    };

    for (int i = 0; i < 4; ++i) {
        Eigen::Matrix3f triangle;
        for (int j = 0; j < 3; ++j) {
            triangle.col(j) = tetrahedron_points[tetrahedron_triangle_indices[i][j]];
        }
        triangles_tetrahedron.push_back(triangle);
    }


    Eigen::Vector3f p(0.0, 0.0, 0.0);
    Eigen::Vector3f closest_point;
    closest_point_on_mesh(p, triangles, closest_point);

    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

    p = Eigen::Vector3f(0.0, 0.0, 1.0);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(0.705, 0.0001));

    p = Eigen::Vector3f(0.3, 0.0, -0.6);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.3, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(-0.6, 0.0001));

    p = Eigen::Vector3f(0.0, 0.0, -1.0);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(-0.705, 0.0001));

    p = Eigen::Vector3f(0.0, 0.5, 0.0);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.035, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

    p = Eigen::Vector3f(0.5, 0.5, 0.0);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.5, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.035, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

    p = Eigen::Vector3f(3.5, 3.5, 1.0);
    closest_point_on_mesh(p, triangles, closest_point);
    REQUIRE_THAT(closest_point.x(), Catch::Matchers::WithinAbs(0.935, 0.0001));
    REQUIRE_THAT(closest_point.y(), Catch::Matchers::WithinAbs(0.035, 0.0001));
    REQUIRE_THAT(closest_point.z(), Catch::Matchers::WithinAbs(0.705, 0.0001));



    BENCHMARK("closest point on tetrahedron (4 faces) "){
        return closest_point_on_mesh(Eigen::Vector3f(0.5, 0.5, 0.0), triangles_tetrahedron, closest_point);
    };


    BENCHMARK("closest point on rectangle (12 faces)"){
    return closest_point_on_mesh(Eigen::Vector3f(0.5, 0.5, 0.0), triangles, closest_point);
    };

    //TODO: add different meshes

}

TEST_CASE("Remote point"){

    Eigen::Vector3f p(0.0, 0.0, 0.0);
    Eigen::Vector3f translation_operator(0.0, 0.0, 0.0);
    Eigen::Matrix3f rotation_operator;
    Eigen::Matrix3f rotation_remote;
    Eigen::Vector3f translation_remote(0.0, 0.0, 0.0);
    Eigen::Vector3f remote_point_result;

    SECTION("Identity"){

        rotation_operator << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;
        rotation_remote << 1.0, 0.0, 0.0,
                0.0, 1.0, 0.0,
                0.0, 0.0, 1.0;

        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);

        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

        p = Eigen::Vector3f(0.0, 0.0, 1.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(1.0, 0.0001));

        p = Eigen::Vector3f(0.3, 0.0, -0.6);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.3, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(-0.6, 0.0001));


    }

    SECTION("rotation only") {
        //90 degree rotation around x axis
        rotation_operator << 1.0, 0.0, 0.0,
                0.0, cos(M_PI/2), -sin(M_PI/2),
                0.0, sin(M_PI/2), cos(M_PI/2);

        rotation_remote.setIdentity();


        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

        p = Eigen::Vector3f(1.0, 0.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));


        p = Eigen::Vector3f(0.0, 1.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(-1.0, 0.0001));


        p = Eigen::Vector3f(1.0, 0.5, 0.5);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.5, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(-0.5, 0.0001));


        rotation_remote << cos(M_PI/2), 0.0, sin(M_PI/2),
                0.0, 1.0, 0.0,
                -sin(M_PI/2), 0.0, cos(M_PI/2);

        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(-0.5, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.5, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(-1.0, 0.0001));

    }

    SECTION("translation only"){
        translation_operator << 1.0, 0.0, 0.0;
        translation_remote << 1.0, 0.0, 0.0;
        rotation_operator.setIdentity();
        rotation_remote.setIdentity();

        p = Eigen::Vector3f(0.0, 0.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

        p = Eigen::Vector3f(0.0, 1.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

        translation_remote << 0.0, 1.0, 0.0;

        p = Eigen::Vector3f(0.0, 0.0, 1.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(-1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(1.0, 0.0001));
    }

    SECTION("rotation + translation"){
        translation_operator << 1.0, 0.0, 0.0;
        translation_remote << 1.0, 0.0, 0.0;
        rotation_operator << 1.0, 0.0, 0.0,
                0.0, cos(M_PI/2), -sin(M_PI/2),
                0.0, sin(M_PI/2), cos(M_PI/2);
        rotation_remote << cos(M_PI/2), 0.0, sin(M_PI/2),
                0.0, 1.0, 0.0,
                -sin(M_PI/2), 0.0, cos(M_PI/2);

        p = Eigen::Vector3f(0.0, 0.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(1.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(0.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(1.0, 0.0001));

        translation_operator << 0.0, 2.0, 6.0;

        p = Eigen::Vector3f(0.0, 1.0, 0.0);
        remote_point(p, translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
        REQUIRE_THAT(remote_point_result.x(), Catch::Matchers::WithinAbs(2.0, 0.0001));
        REQUIRE_THAT(remote_point_result.y(), Catch::Matchers::WithinAbs(-6.0, 0.0001));
        REQUIRE_THAT(remote_point_result.z(), Catch::Matchers::WithinAbs(0.0, 0.0001));

    }

    SECTION("Benchmark") {
        BENCHMARK("remote point"){
        return remote_point(Eigen::Vector3f(0.5, 0.5, 0.0), translation_operator, rotation_operator, translation_remote, rotation_remote, remote_point_result);
    };
    }
}





