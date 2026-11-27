/*
 * STUDENT CODE
 *
 * SeafloorSurvey stores every debris discovery and answers signal-strength
 * queries.
 *
 * Storage: one sorted vector of debris segments per robot path. A robot only
 * ever drives forward, so the debris it discovers arrives in increasing
 * segment order and push_back keeps each vector sorted for free. Counting the
 * debris inside a window [low, high] on one path is then two binary searches,
 * O(log k) for k debris on that path, and memory is one int per discovery.
 */

#pragma once

#include <vector>

#include "mothership.h"

class SeafloorSurvey
{
public:
  // Must be called once before use, with the mothership's robot count.
  void reset(int robotCount);

  void recordDebris(Location location);

  // Number of debris segments within SIGNAL_RANGE of location.segment, along
  // location.robot's path and the paths immediately to its left and right.
  int getSignalStrength(Location location) const;

  // Neighbouring path indices, wrapping around the circle of robots.
  int leftOf(int robot) const;
  int rightOf(int robot) const;

  long long getDebrisCount() const;

  // Debris on one path with segment in [low, high].
  int countInRange(int robot, int low, int high) const;

private:
  std::vector<std::vector<int>> debrisByRobot;
  long long debrisCount = 0;
};
