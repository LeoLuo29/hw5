/*
 * STUDENT CODE - EDIT THIS FILE
 * Keep the class name and required interface.
 * Otherwise, add or change any data members or
 * member functions.
 */

#pragma once

#include <vector>

#include "mothership.h"
#include "robot_selector.h"
#include "seafloor_survey.h"

class Search
{
public:
  explicit Search(Mothership &mothership); // REQUIRED

  SearchResult findShipwreck(); // REQUIRED

private:
  Mothership &mothership; // REQUIRED

  SeafloorSurvey seafloorSurvey;
  RobotSelector robotSelector;

  // Per robot: last safe segment (before its first abyss, within the tether)
  // and current position. Indexed by robot.
  std::vector<int> lastSegment;
  std::vector<int> position;

  long long segmentsExplored = 0;

  void computeLastSegments();
  int signalAt(int robot) const;
  // Depth minus a bonus for excess debris nearby; the sweep advances the
  // robot with the lowest key.
  int sweepKeyOf(int robot) const;
  double backgroundRate() const;
  // Signal at or above which a robot counts as a lead rather than noise.
  int hotThreshold() const;
  // The waiting robot to drive next. Precondition: selector not empty.
  int chooseNextRobot() const;
  // Re-rank `robot`'s waiting neighbours after it found debris or moved.
  void refreshNeighbours(int robot);
};
