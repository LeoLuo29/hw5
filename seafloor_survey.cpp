/*
 * STUDENT CODE
 */

#include "seafloor_survey.h"

#include <algorithm>

void SeafloorSurvey::reset(int robotCount)
{
  debrisByRobot.assign(robotCount, std::vector<int>());
  debrisCount = 0;
}

void SeafloorSurvey::recordDebris(Location location)
{
  std::vector<int> &path = debrisByRobot[location.robot];
  if (path.empty() || path.back() < location.segment)
  {
    // Normal case: robots drive forward, so discoveries arrive in order.
    path.push_back(location.segment);
  }
  else
  {
    // Defensive: keep the vector sorted and free of duplicates regardless.
    auto it = std::lower_bound(path.begin(), path.end(), location.segment);
    if (it != path.end() && *it == location.segment)
    {
      return;
    }
    path.insert(it, location.segment);
  }
  ++debrisCount;
}

int SeafloorSurvey::countInRange(int robot, int low, int high) const
{
  const std::vector<int> &path = debrisByRobot[robot];
  if (path.empty() || path.back() < low || path.front() > high)
  {
    return 0;
  }
  auto first = std::lower_bound(path.begin(), path.end(), low);
  auto last = std::upper_bound(first, path.end(), high);
  return static_cast<int>(last - first);
}

int SeafloorSurvey::leftOf(int robot) const
{
  const int count = static_cast<int>(debrisByRobot.size());
  return robot == 0 ? count - 1 : robot - 1;
}

int SeafloorSurvey::rightOf(int robot) const
{
  const int count = static_cast<int>(debrisByRobot.size());
  return robot + 1 == count ? 0 : robot + 1;
}

int SeafloorSurvey::getSignalStrength(Location location) const
{
  const int low = location.segment - SIGNAL_RANGE;
  const int high = location.segment + SIGNAL_RANGE;
  const int left = leftOf(location.robot);
  const int right = rightOf(location.robot);

  int signal = countInRange(location.robot, low, high);
  // With fewer than three robots a neighbour can coincide with the robot
  // itself (or with the other neighbour); count each path only once.
  if (left != location.robot)
  {
    signal += countInRange(left, low, high);
  }
  if (right != location.robot && right != left)
  {
    signal += countInRange(right, low, high);
  }
  return signal;
}

long long SeafloorSurvey::getDebrisCount() const
{
  return debrisCount;
}
