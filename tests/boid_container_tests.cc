//
// Created by Kaelan Davis on 4/20/2021.
//
#include <catch2/catch.hpp>

#include "core/boid.h"
#include "visualizer/boid_container.h"

TEST_CASE("SeekMouse Test") {
  size_t display_window_width = 0;
  size_t display_window_height = 0;
  size_t num_boids = 3;
  boid_sim::visualizer::BoidContainer container(
      display_window_width, display_window_height, num_boids);
  container.SeekMouse();

  std::vector<boid_sim::Boid> boids = container.boids();

  for (const boid_sim::Boid &boid : boids) {
    REQUIRE(boid.is_seek_mouse());
  }
}

TEST_CASE("DefaultBehavior Test") {
  size_t display_window_width = 0;
  size_t display_window_height = 0;
  size_t num_boids = 3;
  boid_sim::visualizer::BoidContainer container(
      display_window_width, display_window_height, num_boids);
  container.SeekMouse();
  std::vector<boid_sim::Boid> boids = container.boids();

  for (boid_sim::Boid &boid : boids) {
    boid.set_seek_mouse(true);
  }

  container.set_boids(boids);
  container.DefaultBehavior();

  // now use the updated container boids for REQUIRE_FALSE
  for (const boid_sim::Boid &boid : container.boids()) {
    REQUIRE_FALSE(boid.is_seek_mouse());
  }
}