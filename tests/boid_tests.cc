//
// Created by Kaelan Davis on 4/19/2021.
//
#include <catch2/catch.hpp>
#include <cmath>

#include "cinder/gl/gl.h"
#include "core/boid.h"

TEST_CASE("Constructor Tests") {
  glm::vec2 position(1, 1);
  glm::vec2 velocity(0, 0);
  int id = 0;
  float max_speed = 1.0f;
  float fov_radius = 1.0f;
  float body_radius = 1.0f;

  SECTION("Negative Max Speed") {
    max_speed = -1.0f;

    REQUIRE_THROWS_AS(boid_sim::Boid(id, position, velocity, max_speed,
                                     fov_radius, body_radius),
                      std::invalid_argument);
  }

  SECTION("Negative FOV Radius") {
    fov_radius = -1.0f;

    REQUIRE_THROWS_AS(boid_sim::Boid(id, position, velocity, max_speed,
                                     fov_radius, body_radius),
                      std::invalid_argument);
  }

  SECTION("Negative Body Radius") {
    body_radius = -1.0f;

    REQUIRE_THROWS_AS(boid_sim::Boid(id, position, velocity, max_speed,
                                     fov_radius, body_radius),
                      std::invalid_argument);
  }
}

TEST_CASE("Position Updating") {
  int id = 0;
  float max_speed = 1.0f;
  float fov_radius = 0.0f;
  float align_percent = 0.0f;
  float cohesion_percent = 0.0f;
  float separation_percent = 0.0f;

  std::vector<std::vector<float>> container_bounds{{0, 2}, {0, 2}};
  glm::vec2 init_position(1, 1);
  glm::vec2 init_direction(1, 0);

  boid_sim::Boid boid(id, init_position, init_direction, max_speed, fov_radius);
  std::vector<boid_sim::Boid> boids{boid};
  glm::vec2 mouse_pos(0, 0);

  SECTION("Standard Position Update") {
    glm::vec2 velocity(1, 1);
    boid.set_velocity(velocity);
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(1.707f, 1.707f);
    glm::vec2 actual = boid.position();

    REQUIRE(glm::epsilonEqual(expected, actual, .001f)[1]);
  }
}

TEST_CASE("SteerInbounds Out of Bounds Tests") {
  int id = 0;
  float max_speed = 1.0f;
  float fov_radius = 1.0f;
  float align_percent = 0.0f;
  float cohesion_percent = 0.0f;
  float separation_percent = 0.0f;

  std::vector<std::vector<float>> container_bounds{{0, 2}, {0, 2}};
  glm::vec2 init_position(1, 1);
  glm::vec2 init_direction(1, 0);

  boid_sim::Boid boid(id, init_position, init_direction, max_speed, fov_radius);
  std::vector<boid_sim::Boid> boids{boid};
  glm::vec2 mouse_pos(0, 0);

  SECTION("Proper SteerInbounds X-Min") {
    boid.set_position(glm::vec2(-1, 1));
    boid.set_velocity(glm::vec2(-1, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(1, 0);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds X-Max") {
    boid.set_position(glm::vec2(3, 1));
    boid.set_velocity(glm::vec2(1, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(-1, 0);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds Y-Min") {
    boid.set_position(glm::vec2(1, -1));
    boid.set_velocity(glm::vec2(0, -1));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(0, 1);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds Y-Max") {
    boid.set_position(glm::vec2(1, 3));
    boid.set_velocity(glm::vec2(0, 1));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(0, -1);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds X-Min and Y-Min") {
    boid.set_position(glm::vec2(-1, -1));
    boid.set_velocity(glm::vec2(-1, -1));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(.707f, .707f);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .001f)[1]);
  }
}

TEST_CASE("SteerInbounds FOV Radius Tests") {
  int id = 0;
  float max_speed = 1.0f;
  float fov_radius = 2.0f;
  float align_percent = 0.0f;
  float cohesion_percent = 0.0f;
  float separation_percent = 0.0f;

  std::vector<std::vector<float>> container_bounds{{0, 10}, {0, 10}};
  glm::vec2 init_position(1, 1);
  glm::vec2 init_direction(1, 0);

  boid_sim::Boid boid(id, init_position, init_direction, max_speed, fov_radius);
  std::vector<boid_sim::Boid> boids{boid};
  glm::vec2 mouse_pos(0, 0);

  SECTION("Proper SteerInbounds X-Min in FOV") {
    boid.set_position(glm::vec2(1, 5));
    boid.set_velocity(glm::vec2(0, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(1, 0);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds X-Max in FOV") {
    boid.set_position(glm::vec2(9, 5));
    boid.set_velocity(glm::vec2(0, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(-1, 0);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds Y-Min in FOV") {
    boid.set_position(glm::vec2(5, 1));
    boid.set_velocity(glm::vec2(0, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(0, 1);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds Y-Max in FOV") {
    boid.set_position(glm::vec2(5, 9));
    boid.set_velocity(glm::vec2(0, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(0, -1);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .000001f)[1]);
  }

  SECTION("Proper SteerInbounds X-Min and Y-Min in FOV") {
    boid.set_position(glm::vec2(1, 1));
    boid.set_velocity(glm::vec2(0, 0));
    boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                        cohesion_percent, separation_percent);

    glm::vec2 expected(.707f, .707f);
    glm::vec2 actual = boid.velocity();

    REQUIRE(glm::epsilonEqual(expected, actual, .001f)[1]);
  }
}

TEST_CASE("Flocking Rules Tests") {
  int id0 = 0;
  int id1 = 1;
  float max_speed = 1.0f;
  float fov_radius = 2.0f;
  float align_percent = 0.0f;
  float cohesion_percent = 0.0f;
  float separation_percent = 0.0f;

  std::vector<std::vector<float>> container_bounds{{0, 10}, {0, 10}};
  glm::vec2 mouse_pos(0, 0);

  glm::vec2 init_position0(1, 1);
  glm::vec2 init_velocity0(1, 0);

  glm::vec2 init_position1(1, 2);
  glm::vec2 init_velocity(0, 1);

  boid_sim::Boid boid0(id0, init_position0, init_velocity, max_speed,
                       fov_radius);
  boid_sim::Boid boid1(id1, init_position1, init_velocity, max_speed,
                       fov_radius);
  std::vector<boid_sim::Boid> boids{boid0, boid1};

  SECTION("Flocking Rule: Alignment Test") {
    align_percent = 1.0f;
    boid0.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);
    boid1.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);

    glm::vec2 expected0(.275f, .962f);
    glm::vec2 expected1(.371f, .928f);
    glm::vec2 actual0 = boid0.velocity();
    glm::vec2 actual1 = boid1.velocity();

    REQUIRE(glm::epsilonEqual(expected0, actual0, .001f)[1]);
    REQUIRE(glm::epsilonEqual(expected1, actual1, .001f)[1]);
  }

  SECTION("Flocking Rule: Cohesion Test") {
    cohesion_percent = 1.0f;
    boid0.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);
    boid1.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);

    glm::vec2 expected0(.275f, .962f);
    glm::vec2 expected1(.447f, .894f);
    glm::vec2 actual0 = boid0.velocity();
    glm::vec2 actual1 = boid1.velocity();

    REQUIRE(glm::epsilonEqual(expected0, actual0, .001f)[1]);
    REQUIRE(glm::epsilonEqual(expected1, actual1, .001f)[1]);
  }

  SECTION("Flocking Rule: Separation Test") {
    separation_percent = 1.0f;
    boid0.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);
    boid1.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                         cohesion_percent, separation_percent);

    glm::vec2 expected0(.316f, .949f);
    glm::vec2 expected1(.371f, .928f);
    glm::vec2 actual0 = boid0.velocity();
    glm::vec2 actual1 = boid1.velocity();

    REQUIRE(glm::epsilonEqual(expected0, actual0, .001f)[1]);
    REQUIRE(glm::epsilonEqual(expected1, actual1, .001f)[1]);
  }
}

TEST_CASE("Seek Mouse Test") {
  int id = 0;
  float max_speed = 1.0f;
  float fov_radius = 2.0f;
  float align_percent = 0.0f;
  float cohesion_percent = 0.0f;
  float separation_percent = 0.0f;

  std::vector<std::vector<float>> container_bounds{{0, 10}, {0, 10}};
  glm::vec2 mouse_pos(2, 1);

  glm::vec2 init_position(1, 1);
  glm::vec2 init_direction(1, 0);

  boid_sim::Boid boid(id, init_position, init_direction, max_speed, fov_radius);
  boid.set_seek_mouse(true);
  std::vector<boid_sim::Boid> boids{boid};

  boid.UpdatePosition(container_bounds, boids, mouse_pos, align_percent,
                      cohesion_percent, separation_percent);
  glm::vec2 expected(.962, .275);
  glm::vec2 actual = boid.velocity();

  REQUIRE(glm::epsilonEqual(expected, actual, .001f)[1]);
}