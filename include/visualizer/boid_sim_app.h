//
// Created by Kaelan Davis on 4/19/2021.
//
#pragma once

#include "boid_container.h"
#include "cinder/app/App.h"
#include "cinder/app/MouseEvent.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"

namespace boid_sim {

namespace visualizer {

class BoidSimApp : public ci::app::App {
public:
  /**
   * Constructor for BoidSimApp
   */
  BoidSimApp();

  /**
   * Draws all graphics elements to to the screen.
   */
  void draw() override;

  /**
   * Updates all processes that need updating each frame.
   */
  void update() override;

  /**
   * Updates the mouse position based on current mouse location on screen. N
   * updates occur if mouse is not dragged.
   */
  void mouseDrag(ci::app::MouseEvent event) override;

  /**
   * Updates mouse position based on current mouse location on screen. No
   * updates occur during a mouse drag.
   */
  void mouseMove(ci::app::MouseEvent event) override;

  /**
   * Puts boids in "Track Mouse" mode on left click hold.
   */
  void mouseDown(ci::app::MouseEvent event) override;

  /**
   * Takes boids out of "Track Mouse" mode on left click release.
   */
  void mouseUp(ci::app::MouseEvent event) override;

private:
  const size_t kWindowWidth = 1500;
  const size_t kWindowHeight = 900;
  const size_t kNumBoids = 175;

  BoidContainer boid_container_;
  glm::vec2 kMousePos;
};

} // namespace visualizer

} // namespace boid_sim
