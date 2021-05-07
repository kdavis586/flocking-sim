//
// Created by Kaelan Davis on 4/19/2021.
//
#include "visualizer/boid_sim_app.h"
#include "cinder/app/MouseEvent.h"

namespace boid_sim {

namespace visualizer {

BoidSimApp::BoidSimApp() {
  ci::app::setWindowSize(kWindowWidth, kWindowHeight);
  boid_container_ = BoidContainer(kWindowWidth, kWindowHeight, kNumBoids);
}

void BoidSimApp::draw() {
  ci::gl::clear(ci::Color("Black"));
  boid_container_.Display();
}

void BoidSimApp::update() { boid_container_.AdvanceOnFrame(kMousePos); }

void BoidSimApp::mouseDrag(ci::app::MouseEvent event) { mouseMove(event); }

void BoidSimApp::mouseMove(ci::app::MouseEvent event) {
  kMousePos = event.getPos();
}

void BoidSimApp::mouseDown(ci::app::MouseEvent event) {
  if (event.isLeft()) {
    boid_container_.SeekMouse();
  }
}
void BoidSimApp::mouseUp(ci::app::MouseEvent event) {
  if (event.isLeft()) {
    boid_container_.DefaultBehavior();
  }
}

} // namespace visualizer

} // namespace boid_sim