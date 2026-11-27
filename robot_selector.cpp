/*
 * STUDENT CODE
 */

#include "robot_selector.h"

void RobotSelector::reset(int robotCount)
{
  bySignal.clear();
  bySweep.clear();
  signalOf.assign(robotCount, 0);
  depthOf.assign(robotCount, 0);
  sweepKeyOf.assign(robotCount, 0);
  present.assign(robotCount, false);
}

void RobotSelector::add(int robot, int signal, int depth, int sweepKey)
{
  if (present[robot])
  {
    remove(robot);
  }
  signalOf[robot] = signal;
  depthOf[robot] = depth;
  sweepKeyOf[robot] = sweepKey;
  present[robot] = true;
  if (signal > 0)
  {
    bySignal.insert(SignalKey{signal, depth, robot});
  }
  bySweep.insert(SweepKey{sweepKey, robot});
}

void RobotSelector::remove(int robot)
{
  if (!present[robot])
  {
    return;
  }
  if (signalOf[robot] > 0)
  {
    bySignal.erase(SignalKey{signalOf[robot], depthOf[robot], robot});
  }
  bySweep.erase(SweepKey{sweepKeyOf[robot], robot});
  present[robot] = false;
}

void RobotSelector::update(int robot, int signal, int sweepKey)
{
  if (!present[robot])
  {
    return;
  }
  // Erase, modify, re-insert: ordered-set keys are not mutable in place.
  if (signalOf[robot] != signal)
  {
    if (signalOf[robot] > 0)
    {
      bySignal.erase(SignalKey{signalOf[robot], depthOf[robot], robot});
    }
    signalOf[robot] = signal;
    if (signal > 0)
    {
      bySignal.insert(SignalKey{signal, depthOf[robot], robot});
    }
  }
  if (sweepKeyOf[robot] != sweepKey)
  {
    bySweep.erase(SweepKey{sweepKeyOf[robot], robot});
    sweepKeyOf[robot] = sweepKey;
    bySweep.insert(SweepKey{sweepKey, robot});
  }
}

bool RobotSelector::contains(int robot) const
{
  return present[robot];
}

bool RobotSelector::empty() const
{
  return bySweep.empty();
}

int RobotSelector::strongestRobot() const
{
  return bySignal.begin()->robot;
}

int RobotSelector::strongestSignal() const
{
  return bySignal.empty() ? 0 : bySignal.begin()->signal;
}

int RobotSelector::sweepRobot() const
{
  return bySweep.begin()->robot;
}

int RobotSelector::lowestSweepKey() const
{
  return bySweep.begin()->key;
}
