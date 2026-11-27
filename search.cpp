/*
 * STUDENT CODE - EDIT THIS FILE
 *
 * Search heuristic: "sweep, lean toward evidence, follow strong leads".
 *
 * 1. Sweep. With no strong lead anywhere, drive the robot with the lowest
 *    sweep key, in bursts: it keeps driving until it is kSweepBurst segments
 *    ahead of the next robot in line. Every unexplored segment is equally
 *    likely to hold the wreck a priori and debris fields are wide, so a
 *    breadth-first sweep reaches the first debris field early. Bursts
 *    amortise the 200-tick switch cost: switching every 100 segments (round
 *    robin) wastes 2/3 of the time; switching every kSweepBurst segments
 *    wastes only 200 / kSweepBurst.
 *
 * 2. Lean toward evidence. The sweep key is a robot's depth minus a bonus for
 *    excess debris in a wide window (+/- kEvidenceRange) on its path and both
 *    neighbours: (found - expected background) / sqrt(expected), in standard
 *    deviations. Weak, spread-out debris around a wreck site is invisible in
 *    the 500-segment signal window but statistically clear over thousands of
 *    segments, so robots in such fields run ahead of the sweep.
 *
 * 3. Follow strong leads. A robot whose signal strength (the assignment's
 *    +/- 500 definition) is well above what background debris would produce
 *    (hotThreshold) is a lead. The strongest lead is driven immediately and
 *    stays active while it remains the strongest (with a small hysteresis
 *    margin, since switching costs 200 ticks). Debris found by one robot
 *    raises its neighbours' scores too, so a debris field pulls in all three
 *    paths it spans.
 *
 * 4. Decoys (former wreck sites) handle themselves: once a robot has driven
 *    past a debris field, the field leaves its windows, the robot goes cold,
 *    and the sweep resumes from the shallowest robots.
 *
 * Thresholds adapt to the observed background debris rate, so the heuristic
 * does not depend on one particular debris density. The constants below were
 * chosen from a plateau of good settings, not a single best point.
 *
 * Data structures: see seafloor_survey.h (debris) and robot_selector.h
 * (choosing the next robot).
 */

#include "search.h"

#include <algorithm>
#include <cmath>

namespace
{
  // How far (in sweep-key segments) a cold robot runs ahead of the next
  // robot in line before the sweep moves on.
  constexpr int kSweepBurst = 7000;
  // A signal this many standard deviations above the background mean is hot.
  constexpr double kHotSigmas = 3.0;
  // Never treat fewer than this many nearby debris as a lead.
  constexpr int kMinHotSignal = 4;
  // A hot robot keeps driving unless another lead is stronger by this much.
  constexpr int kHotSwitchMargin = 3;
  // Number of segments in one robot's signal window, on one path.
  constexpr int kWindowLength = 2 * SIGNAL_RANGE + 1;
  // Half-width of the wider window used to weigh weak, spread-out evidence.
  constexpr int kEvidenceRange = 10000;
  // How many segments ahead of the sweep one standard deviation of excess
  // debris lets a robot run.
  constexpr double kLeadPerSigma = 20000;
}

Search::Search(Mothership &mothership)
    : mothership(mothership)
{
}

void Search::computeLastSegments()
{
  // A robot can never pass the first abyss on its path, so only the nearest
  // abyss per robot matters: one pass over the unordered list, O(A + N).
  const int robotCount = mothership.getRobotCount();
  lastSegment.assign(robotCount, TETHER_LENGTH);
  for (const Location &abyss : mothership.getAbysses())
  {
    if (abyss.robot < 0 || abyss.robot >= robotCount)
    {
      continue;
    }
    lastSegment[abyss.robot] = std::min(lastSegment[abyss.robot], abyss.segment - 1);
  }
}

int Search::signalAt(int robot) const
{
  return seafloorSurvey.getSignalStrength(Location{robot, position[robot]});
}

double Search::backgroundRate() const
{
  // Estimated debris per segment, from everything seen so far (one
  // pseudo-observation per 1000 segments keeps early estimates sane).
  return (seafloorSurvey.getDebrisCount() + 1.0) /
         (static_cast<double>(segmentsExplored) + 1000.0);
}

int Search::sweepKeyOf(int robot) const
{
  // Compare the debris found in a wide window around the robot's position,
  // on its own path and both neighbours, with what background debris alone
  // would leave on the explored part of that window. The excess, in standard
  // deviations, lets the robot run ahead of the sweep: weak but wide debris
  // fields (e.g. the outskirts of a wreck site) are invisible to the 500-
  // segment signal window but clear over a few thousand segments.
  const int depth = position[robot];
  const int low = depth - kEvidenceRange;
  const int high = depth + kEvidenceRange;
  const int paths[3] = {seafloorSurvey.leftOf(robot), robot, seafloorSurvey.rightOf(robot)};

  int found = 0;
  long long explored = 0;
  for (int i = 0; i < 3; ++i)
  {
    const int path = paths[i];
    if ((i == 0 && path == robot) || (i == 2 && (path == robot || path == paths[0])))
    {
      continue; // fewer than three robots
    }
    found += seafloorSurvey.countInRange(path, low, high);
    // Segments 1..position[path] of this path have been explored.
    explored += std::max(0, std::min(high, position[path]) - std::max(1, low) + 1);
  }

  const double expected = backgroundRate() * static_cast<double>(explored);
  const double sigmas = (found - expected) / std::sqrt(expected + 1.0);
  // Capped so extreme debris densities cannot overflow the int key.
  const double bonus = std::min(kLeadPerSigma * std::max(0.0, sigmas),
                                static_cast<double>(TETHER_LENGTH));
  return depth - static_cast<int>(bonus);
}

int Search::hotThreshold() const
{
  // Expected signal from background debris alone over three full windows.
  const double mean = 3.0 * kWindowLength * backgroundRate();
  const int threshold = static_cast<int>(std::ceil(mean + kHotSigmas * std::sqrt(mean)));
  return std::max(kMinHotSignal, threshold);
}

int Search::chooseNextRobot() const
{
  // hotThreshold() is always positive, so a hot signal implies one exists.
  if (robotSelector.strongestSignal() >= hotThreshold())
  {
    return robotSelector.strongestRobot();
  }
  return robotSelector.sweepRobot();
}

void Search::refreshNeighbours(int robot)
{
  const int left = seafloorSurvey.leftOf(robot);
  const int right = seafloorSurvey.rightOf(robot);
  if (left != robot && robotSelector.contains(left))
  {
    robotSelector.update(left, signalAt(left), sweepKeyOf(left));
  }
  if (right != robot && right != left && robotSelector.contains(right))
  {
    robotSelector.update(right, signalAt(right), sweepKeyOf(right));
  }
}

SearchResult Search::findShipwreck()
{
  const int robotCount = mothership.getRobotCount();
  computeLastSegments();
  seafloorSurvey.reset(robotCount);
  robotSelector.reset(robotCount);
  position.assign(robotCount, 0);

  for (int robot = 0; robot < robotCount; ++robot)
  {
    position[robot] = mothership.getRobotPosition(robot);
    if (position[robot] < lastSegment[robot])
    {
      robotSelector.add(robot, 0, position[robot], position[robot]);
    }
  }

  int current = -1; // robot being driven, checked out of the selector

  while (true)
  {
    if (current < 0)
    {
      if (robotSelector.empty())
      {
        // Every reachable segment of every path has been driven.
        return SearchResult{false, Location{}};
      }
      current = chooseNextRobot();
      robotSelector.remove(current);
      mothership.switchToRobot(current); // free if already active
    }

    const int distance = std::min(MAX_SEGMENTS_PER_DRIVE, lastSegment[current] - position[current]);
    const DriveResult result = mothership.driveActiveRobot(distance);
    position[current] += result.segmentsMoved;
    segmentsExplored += result.segmentsMoved;

    if (result.reason == DriveStopReason::ShipwreckFound)
    {
      return SearchResult{true, Location{current, position[current]}};
    }
    if (result.reason == DriveStopReason::DebrisFound)
    {
      seafloorSurvey.recordDebris(Location{current, position[current]});
      refreshNeighbours(current);
    }

    if (position[current] >= lastSegment[current])
    {
      refreshNeighbours(current);
      current = -1; // path exhausted; never re-added
      continue;
    }

    // Decide whether to keep driving the current robot.
    const int signal = signalAt(current);
    const int threshold = hotThreshold();
    const int bestOther = robotSelector.strongestSignal();

    bool keep;
    if (signal >= threshold)
    {
      keep = signal + kHotSwitchMargin >= bestOther;
    }
    else if (bestOther >= threshold)
    {
      keep = false;
    }
    else
    {
      // Sweep: run until a full burst ahead of the next robot in line.
      keep = robotSelector.empty() ||
             sweepKeyOf(current) < robotSelector.lowestSweepKey() + kSweepBurst;
    }

    if (!keep)
    {
      robotSelector.add(current, signal, position[current], sweepKeyOf(current));
      refreshNeighbours(current); // their evidence windows changed
      current = -1;
    }
  }
}
