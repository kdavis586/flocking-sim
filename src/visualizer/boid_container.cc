//
// Created by Kaelan Davis on 4/19/2021.
//
#include <random>

#include "visualizer/boid_container.h"

namespace boid_sim {

namespace visualizer {

BoidContainer::BoidContainer() = default;

BoidContainer::BoidContainer(size_t display_window_width,
                             size_t display_window_height, size_t num_boids)
    : num_boids_(num_boids) {
  SetContainerBounds(display_window_width, display_window_height);
  PopulateBoids();
}

BoidContainer &BoidContainer::operator=(const BoidContainer &source) {
  boids_ = source.boids_;
  container_bounds_ = source.container_bounds_;
  num_boids_ = source.num_boids_;

  return *this;
}

void BoidContainer::SetContainerBounds(size_t display_window_width,
                                       size_t display_window_height) {
  std::vector<float> x_bounds{0.0f, (float)display_window_width};
  std::vector<float> y_bounds{0.0f, (float)display_window_height};

  container_bounds_ = {x_bounds, y_bounds};
}

void BoidContainer::Display() {
  for (Boid &boid : boids_) {
    boid.Draw();
  }
}

void BoidContainer::PopulateBoids() {
  for (size_t i = 0; i < num_boids_; i++) {
    glm::vec2 start_position = GenerateRandomPosition();
    glm::vec2 start_direction = GenerateRandomDirection();

    float boid_speed = 2.0f;
    Boid boid(i, start_position, start_direction, boid_speed);

    boids_.push_back(boid);
  }
}

void BoidContainer::AdvanceOnFrame(glm::vec2 &mouse_pos) {
  /*
   * All calculations for all boids use this "snapshot" of time.
   *  So updated boids don't affect calculations of boids that still
   *  need to be updated.
   */

  std::vector<Boid> boid_snapshot = boids_;

  float align_percent = .30f;
  float cohesion_percent = .95f;
  float separation_percent = 1.0f;

  for (Boid &boid : boids_) {
    boid.UpdatePosition(container_bounds_, boid_snapshot, mouse_pos,
                        align_percent, cohesion_percent, separation_percent);
  }
}

void BoidContainer::SeekMouse() {
  for (Boid &boid : boids_) {
    boid.set_seek_mouse(true);
  }
}

void BoidContainer::DefaultBehavior() {
  for (Boid &boid : boids_) {
    boid.set_seek_mouse(false);
  }
}

glm::vec2 BoidContainer::GenerateRandomDirection() {
  std::random_device rd;
  std::uniform_real_distribution<float> direction_distribution(-1.f, 1.f);

  // Vector with velocity 1 and random direction
  glm::vec2 normalized_random_direction = glm::normalize(
      glm::vec2(direction_distribution(rd), direction_distribution(rd)));

  return normalized_random_direction;
}

glm::vec2 BoidContainer::GenerateRandomPosition() {
  std::random_device rd;
  std::uniform_real_distribution<float> x_bounds_distribution(
      container_bounds_[0][0], container_bounds_[0][1]);
  std::uniform_real_distribution<float> y_bounds_distribution(
      container_bounds_[1][0], container_bounds_[1][1]);

  return glm::vec2(x_bounds_distribution(rd), y_bounds_distribution(rd));
}

const std::vector<boid_sim::Boid> &BoidContainer::boids() const {
  return boids_;
}
void BoidContainer::set_boids(const std::vector<boid_sim::Boid> &boids) {
  boids_ = boids;
}

} // namespace visualizer

} // namespace boid_sim
