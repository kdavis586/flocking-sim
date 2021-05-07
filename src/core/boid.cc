//
// Created by Kaelan Davis on 4/19/2021.
//

#include <cmath>

#include "core/boid.h"

namespace boid_sim {
boid_sim::Boid::Boid(int id, glm::vec2 &position, glm::vec2 &direction,
                     float max_speed, float fov_radius, float body_radius)
    : id_(id), position_(position) {
  ValidateValues(max_speed, fov_radius, body_radius);
  max_speed_ = max_speed;
  fov_radius_ = fov_radius;
  body_radius_ = body_radius;
  seek_mouse_ = false;
  velocity_ = glm::normalize(direction) * max_speed_;
  float max_for_speed_percent = .20f;
  max_force_ = max_for_speed_percent * max_speed_;
}

Boid &Boid::operator=(const Boid &source) {
  body_radius_ = source.body_radius_;
  max_speed_ = source.max_speed_;
  max_force_ = source.max_force_;
  fov_radius_ = source.max_force_;
  id_ = source.id_;
  position_ = source.position_;
  velocity_ = source.velocity_;

  return *this;
}

bool operator!=(const Boid &boid1, const Boid &boid2) {
  if (boid1.id_ != boid2.id_) {
    return true;
  } else if (boid1.position_ != boid2.position_) {
    return true;
  } else if (boid1.velocity_ != boid2.velocity_) {
    return true;
  }

  return false;
}

void Boid::Draw() {
  std::vector<glm::vec2> vertices = CalculateVertices();
  ci::gl::color(ci::Color("MediumAquamarine"));
  ci::gl::drawSolidTriangle(vertices[0], vertices[1], vertices[2]);

  ci::gl::color(ci::Color("Red"));
  float nose_radius = 4.0f;
  ci::gl::drawSolidCircle(vertices[0], nose_radius);
}

void Boid::UpdatePosition(std::vector<std::vector<float>> &container_bounds,
                          std::vector<Boid> &boids, glm::vec2 &mouse_pos,
                          float align_percent, float cohesion_percent,
                          float separation_percent) {
  glm::vec2 acceleration = Flock(boids, mouse_pos, align_percent,
                                 cohesion_percent, separation_percent) +
                           SteerInbounds(container_bounds);

  if (seek_mouse_) {
    acceleration += Seek(mouse_pos);
  }

  velocity_ += acceleration;
  velocity_ = glm::normalize(velocity_) * max_speed_;

  position_ += velocity_;
}

glm::vec2 Boid::Flock(std::vector<Boid> &boids, glm::vec2 &mouse_pos,
                      float align_percent, float cohesion_percent,
                      float separation_percent) {
  std::vector<Boid> boids_in_fov = GetBoidsInVision(boids);

  glm::vec2 align_force = Align(boids_in_fov);
  glm::vec2 cohes_force = Cohesion(boids_in_fov);
  glm::vec2 sep_force = Separation(boids_in_fov);

  glm::vec2 accel_force =
      (align_force * align_percent + cohes_force * cohesion_percent +
       sep_force * separation_percent);

  if (glm::length(accel_force) > max_force_) {
    accel_force = glm::normalize(accel_force) * max_force_;
  }

  return accel_force;
}

glm::vec2 Boid::Align(std::vector<Boid> &boids) {
  glm::vec2 desired_direction(0, 0);

  for (const Boid &boid : boids) {
    desired_direction += boid.velocity_;
  }

  desired_direction /= (float)boids.size();
  glm::vec2 steer_force = CalcSteerForce(desired_direction);

  return steer_force;
}

glm::vec2 Boid::Cohesion(std::vector<Boid> &boids) {
  glm::vec2 avg_position(0, 0);
  glm::vec2 steer_force(0, 0);

  for (const Boid &boid : boids) {
    avg_position += boid.position();
  }

  if (avg_position != position_) {
    avg_position /= (float)boids.size();
    glm::vec2 desired_direction = avg_position - position_;

    steer_force = CalcSteerForce(desired_direction);
  }

  return steer_force;
}

glm::vec2 Boid::Separation(std::vector<Boid> &boids) {
  glm::vec2 desired_direction(0, 0);
  glm::vec2 steer_force(0, 0);
  size_t total = 0;

  for (const Boid &boid : boids) {
    float distance = glm::distance(position_, boid.position());

    if (distance > 0) {
      glm::vec2 direction_away = position_ - boid.position();
      direction_away = glm::normalize(direction_away);
      direction_away /= (distance / fov_radius_);
      desired_direction += direction_away;
      total++;
    }
  }

  if (total > 0) {
    desired_direction /= (float)total;
    steer_force = CalcSteerForce(desired_direction);
  }

  return steer_force;
}

glm::vec2 Boid::Seek(glm::vec2 &desired_position) {
  float distance = glm::distance(desired_position, position_);
  glm::vec2 steer_force;

  if (distance < fov_radius_) {
    glm::vec2 desired_direction = desired_position - position_;
    steer_force = CalcSteerForce(desired_direction);
  }

  if (glm::length(steer_force) > max_force_) {
    steer_force = glm::normalize(steer_force) * max_force_;
  }

  return steer_force;
}

glm::vec2
Boid::SteerInbounds(std::vector<std::vector<float>> &container_bounds) {
  glm::vec2 steering_force(0, 0);

  FixZeroComponentVelocity();
  steering_force[0] = HandleHorizontalBounds(container_bounds);
  steering_force[1] = HandleVerticalBounds(container_bounds);

  return steering_force;
}

float Boid::HandleHorizontalBounds(
    std::vector<std::vector<float>> &container_bounds) const {
  float x_component = 0.0f;

  float x_min_bound = container_bounds[0][0];
  float x_max_bound = container_bounds[0][1];
  float dist_x_min = std::abs(position_.x - x_min_bound);
  float dist_x_max = std::abs(position_.x - x_max_bound);

  bool min_bound_in_fov = dist_x_min < fov_radius_;
  bool max_bound_in_fov = dist_x_max < fov_radius_;
  bool out_of_min_bound = position_.x <= x_min_bound;
  bool out_of_max_bound = position_.x >= x_max_bound;

  if (min_bound_in_fov || out_of_min_bound) {
    x_component = max_force_ / (dist_x_min / fov_radius_);

    if (out_of_min_bound) {
      // Off screen, make steering force component massive;
      x_component = 1.0f / kEpsilon;
    }
  } else if (max_bound_in_fov || out_of_max_bound) {
    x_component = -max_force_ / (dist_x_max / fov_radius_);

    if (out_of_max_bound) {
      // Off screen, make steering force component massive;
      x_component = -1.0f / kEpsilon;
    }
  }

  return x_component;
}

float Boid::HandleVerticalBounds(
    std::vector<std::vector<float>> &container_bounds) const {
  float y_component = 0.0f;

  float y_min_bound = container_bounds[1][0];
  float y_max_bound = container_bounds[1][1];
  float dist_y_min = std::abs(position_.y - y_min_bound);
  float dist_y_max = std::abs(position_.y - y_max_bound);

  bool min_bound_in_fov = dist_y_min < fov_radius_;
  bool max_bound_in_fov = dist_y_max < fov_radius_;
  bool out_of_min_bound = position_.y <= y_min_bound;
  bool out_of_max_bound = position_.y >= y_max_bound;

  if (min_bound_in_fov || out_of_min_bound) {
    y_component = max_force_ / (dist_y_min / fov_radius_);

    if (out_of_min_bound) {
      // Off screen, make steering force component massive;
      y_component = 1.0f / kEpsilon;
    }
  } else if (max_bound_in_fov || out_of_max_bound) {
    y_component = -max_force_ / (dist_y_max / fov_radius_);

    if (out_of_max_bound) {
      // Off screen, make steering force component massive;
      y_component = -1.0f / kEpsilon;
    }
  }

  return y_component;
}

glm::vec2 Boid::CalcSteerForce(glm::vec2 &desired_direction) {
  glm::vec2 steer_force(0, 0);

  if (glm::length(desired_direction) > 0) {
    steer_force = glm::normalize(desired_direction);
    steer_force *= max_speed_;
    steer_force -= velocity_;
  }

  return steer_force;
}

std::vector<glm::vec2> Boid::CalculateVertices() {
  float num_vertices = 3.0f;
  std::vector<glm::vec2> vertices;
  float start_angle = GetVelocityAngle();

  for (size_t i = 0; i < num_vertices; i++) {
    float adjust_angle = 2.0f * (float)M_PI * i / num_vertices;

    float x =
        position_[0] + body_radius_ * std::cos(start_angle + adjust_angle);
    float y =
        position_[1] + body_radius_ * std::sin(start_angle + adjust_angle);

    glm::vec2 vertex(x, y);
    vertices.push_back(vertex);
  }

  return vertices;
}

std::vector<Boid> Boid::GetBoidsInVision(const std::vector<Boid> &boids) {
  std::vector<Boid> boids_in_fov;

  for (const Boid &boid : boids) {
    float distance = glm::distance(position_, boid.position());

    if (distance < fov_radius_ && *this != boid) {
      boids_in_fov.push_back(boid);
    }
  }

  return boids_in_fov;
}

float Boid::GetVelocityAngle() {
  float angle;
  if (velocity_[0] == 0.0f) {
    angle = (velocity_[1] > 0.0f ? 1.0f : -1.0f) * (float)M_PI / 2.0f;
  } else {
    angle = std::atan(velocity_[1] / velocity_[0]);
  }

  /* std::atan only gives radian angles in quadrant I and IV, so directions in
   * quadrant and II and III are lost.
   *
   * if the x-component of velocity is zero, degree produced from atan
   * needs to be in either II or III, so PI is added to angle to correct
   * start_angle.
   */
  if (velocity_[0] < 0) {
    angle += (float)M_PI;
  }

  return angle;
}

void Boid::FixZeroComponentVelocity() {
  /*
   * Boid algorithm works in such a way that boids with perfect 0 velocity
   * components will ignore their components respective bounds. Epsilon is added
   * to fix this.
   */
  if (velocity_[0] == 0.0f) {
    velocity_[0] = kEpsilon;
  }

  if (velocity_[1] == 0.0f) {
    velocity_[1] = kEpsilon;
  }
}

void Boid::ValidateValues(float max_speed, float fov_radius,
                          float body_radius) {
  if (max_speed < 0.0f) {
    throw std::invalid_argument("Max speed was less than 0!");
  } else if (fov_radius < 0.0f) {
    throw std::invalid_argument("FOV radius was less than 0!");
  } else if (body_radius < 0.0f) {
    throw std::invalid_argument("Body radius was less than 0!");
  }
}

const glm::vec2 &Boid::position() const { return position_; }

void Boid::set_position(const glm::vec2 &position) { position_ = position; }

const glm::vec2 &Boid::velocity() const { return velocity_; }

void Boid::set_velocity(const glm::vec2 &velocity) { velocity_ = velocity; }

bool Boid::is_seek_mouse() const { return seek_mouse_; }

void Boid::set_seek_mouse(bool seek_mouse) { seek_mouse_ = seek_mouse; }
} // namespace boid_sim
