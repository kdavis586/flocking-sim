//
// Created by Kaelan Davis on 4/19/2021.
//
#include <iostream>

#include "visualizer/boid_sim_app.h"

using boid_sim::visualizer::BoidSimApp;

void prepareSettings(BoidSimApp::Settings *settings) {
  settings->setResizable(false);
}

CINDER_APP(BoidSimApp, ci::app::RendererGl, prepareSettings);