/*
 * STUDENT CODE
 *
 * RobotSelector holds every robot that can still drive (not at its abyss or
 * tether end) and is not the robot currently being driven, ordered two ways:
 *
 *   bySignal - strongest signal first (ties: shallowest, then lowest index).
 *              begin() is the most promising lead. Only robots with a
 *              nonzero signal are kept here (a zero signal is never a lead),
 *              which keeps this tree small when most paths are quiet.
 *   bySweep  - lowest sweep key first (ties: lowest index). The sweep key
 *              is the robot's depth, minus a bonus for debris evidence, so
 *              begin() is where the breadth-first sweep should continue.
 *
 * Both are AVL trees, so the best robot of either kind is found in O(1)
 * (begin()) and a robot is added, removed or re-ranked in O(log N). The
 * driven robot is "checked out" of both trees while it drives, so the
 * common case (driving the same robot again) costs no tree operations;
 * only debris discoveries (re-ranking up to two neighbours) and switches
 * touch the trees. This keeps the 100k-robot case fast.
 *
 * Re-ranking uses the erase / modify / insert pattern, since ordered set
 * elements cannot be changed in place.
 */

#pragma once

#include <vector>

#include "trees.h"

class RobotSelector
{
public:
  void reset(int robotCount);

  // Add an available robot with its signal, depth and sweep key.
  void add(int robot, int signal, int depth, int sweepKey);
  // Remove a robot (e.g. to drive it). No-op if not present.
  void remove(int robot);
  // Change the signal and sweep key of a waiting robot. No-op if absent.
  void update(int robot, int signal, int sweepKey);

  bool contains(int robot) const;
  bool empty() const;

  // strongestSignal() is 0 when no waiting robot has any signal.
  int strongestSignal() const;
  // Precondition: strongestSignal() > 0.
  int strongestRobot() const;
  // Precondition: !empty().
  int sweepRobot() const;
  int lowestSweepKey() const;

private:
  struct SignalKey
  {
    int signal;
    int depth;
    int robot;

    bool operator<(const SignalKey &other) const
    {
      if (signal != other.signal)
      {
        return signal > other.signal; // stronger first
      }
      if (depth != other.depth)
      {
        return depth < other.depth; // then shallower
      }
      return robot < other.robot;
    }
  };

  struct SweepKey
  {
    int key;
    int robot;

    bool operator<(const SweepKey &other) const
    {
      if (key != other.key)
      {
        return key < other.key; // lowest (shallowest, adjusted) first
      }
      return robot < other.robot;
    }
  };

  AVLSet<SignalKey> bySignal;
  AVLSet<SweepKey> bySweep;

  // The key each waiting robot is stored under, so it can be erased exactly.
  std::vector<int> signalOf;
  std::vector<int> depthOf;
  std::vector<int> sweepKeyOf;
  std::vector<bool> present;
};
