# Boids Flocking Algorithm

## *What is it?*

The boids flocking algorithm was originally developed by Craig Reynolds in 1986. While implementations and dimensions (
2D vs 3D) may vary, the overarching concept is the concept of emergent behavior coming from simple rules applied to
virtual agents.

In this case, the agents in question are boids. Boids are bird-like objects that move around the screen with a position,
velocity, and acceleration. Emergent, flocking-like, behavior appears in groups of these boids when 3 simple rules are
applied to them.

1. Alignment
    1. Boids want to move to try to align their direction of travel to the average direction of travel of boids around
       them.
2. Cohesion
    1. Boids want to move to try to be in the center of surrounding boids. Another way to word this is that boids want
       to try to move to the average position of neighboring boids.
3. Separation
    1. Boids want to try to move away from each other. This rule to used to ensure that while boids want to be close,
       they dont want to be close to the point where they like to overlap, taking away from the "lifelike" behavior of
       the algorithm.

Typically, boids also have some sort of "vision cone" or "vision radius", depending on how complex any particular
implementation is. This is done to more accurately replicate the idea birds or fish only keeping in mind the fish
directly around them.
---

# How do I use this visualization?

My particular implementation depends on the Cinder graphics library to run and display the visualization of the boids.
In order to use this visualization on your machine you will need to go ahead and download the Cinder library from their
website. After that, you will want to create a folder under Cinder, then place this project in that folder. From there,
building the project and running it should work!
---
__NOTE__

*I have not tested the instructions above so there is a chance that the exact instructions may not work. However, trying
different versions of Cinder or naming the subfolder you placed in the project in "my-projects" may help.*
