//
// Created by Kaelan Davis on 4/19/2021.
//
#pragma once

#include "cinder/gl/gl.h"

namespace boid_sim {

class Boid {
public:
  /**
   * Constructor for Boid
   */
  Boid(int id, glm::vec2 &position, glm::vec2 &direction,
       float max_speed = 2.0f, float fov_radius = 85.0f,
       float body_radius_ = 6.0f);

  /**
   * Copy assignment constructor
   */
  Boid &operator=(const Boid &source);

  friend bool operator!=(const Boid &boid1, const Boid &boid2);

  /**
   * Updates the position coordinates of the boid
   */
  void UpdatePosition(std::vector<std::vector<float>> &container_bounds,
                      std::vector<Boid> &boids, glm::vec2 &mouse_pos,
                      float align_percent, float cohesion_percent,
                      float separation_percent);

  /**
   * Draws boid to the screen
   */
  void Draw();

  const glm::vec2 &position() const;

  void set_position(const glm::vec2 &position);

  const glm::vec2 &velocity() const;

  void set_velocity(const glm::vec2 &velocity);

  bool is_seek_mouse() const;
  
  void set_seek_mouse(bool seek_mouse);

private:
  const float kEpsilon = 0.00000000001f;

  // how far away each triangle vertex is from the center of boid
  float body_radius_;
  float max_speed_;
  float max_force_;
  float fov_radius_;
  int id_;
  bool seek_mouse_;
  glm::vec2 position_;
  glm::vec2 velocity_;

  std::vector<glm::vec2> CalculateVertices();
  glm::vec2 SteerInbounds(std::vector<std::vector<float>> &container_bounds);
  glm::vec2 Flock(std::vector<Boid> &boids, glm::vec2 &mouse_pos,
                  float align_percent, float cohesion_percent,
                  float separation_percent);
  glm::vec2 Align(std::vector<Boid> &boids);
  glm::vec2 Cohesion(std::vector<Boid> &boids);
  glm::vec2 Separation(std::vector<Boid> &boids);
  glm::vec2 Seek(glm::vec2 &desired_position);
  glm::vec2 CalcSteerForce(glm::vec2 &desired_direction);
  std::vector<Boid> GetBoidsInVision(const std::vector<Boid> &boids);
  void FixZeroComponentVelocity();
  float HandleHorizontalBounds(
      std::vector<std::vector<float>> &container_bounds) const;
  float
  HandleVerticalBounds(std::vector<std::vector<float>> &container_bounds) const;
  float GetVelocityAngle();
  void ValidateValues(float max_speed, float fov_radius, float body_radius);
};

} // namespace boid_sim
