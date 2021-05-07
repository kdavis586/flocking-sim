//
// Created by Kaelan Davis on 4/19/2021.
//
#pragma once

#include <vector>

#include "core/boid.h"

namespace boid_sim {

namespace visualizer {

class BoidContainer {
public:
  /**
   * Default Constructor for BoidContainer
   */
  BoidContainer();

  /**
   * Constructor for BoidContainer
   */
  BoidContainer(size_t display_window_width, size_t display_window_height,
                size_t num_boids);

  /**
   * Copy assignment operator
   */
  BoidContainer &operator=(const BoidContainer &source);

  /**
   * Displays all of current positions of the boids on the screen
   */
  void Display();

  /**
   * Updates the positions and velocities of all boids based on the three rules
   * of cohesion, separation, and alignment
   */
  void AdvanceOnFrame(glm::vec2 &mouse_pos);

  /**
   * Sets all of the boids to "Seek Mouse" mode.
   */
  void SeekMouse();

  /**
   * Sets all of the boids to default behavior.
   */
  void DefaultBehavior();

  const std::vector<boid_sim::Boid> &boids() const;

  void set_boids(const std::vector<boid_sim::Boid> &boids);

private:
  std::vector<std::vector<float>> container_bounds_;
  size_t num_boids_;
  std::vector<boid_sim::Boid> boids_;

  void SetContainerBounds(size_t display_window_width,
                          size_t display_window_height);

  void PopulateBoids();

  static glm::vec2 GenerateRandomDirection();

  glm::vec2 GenerateRandomPosition();
};

} // namespace visualizer

} // namespace boid_sim
