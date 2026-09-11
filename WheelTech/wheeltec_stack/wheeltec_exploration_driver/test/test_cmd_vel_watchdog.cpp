#include <chrono>
#include <limits>

#include <gtest/gtest.h>

#include "wheeltec_robot.h"

namespace {

using Milliseconds = std::chrono::milliseconds;

TEST(CmdVelWatchdogTest, StartsLockedAtZero)
{
  CmdVelWatchdog watchdog;
  const auto now = CmdVelWatchdog::TimePoint(Milliseconds(1000));
  bool fresh = true;

  const geometry_msgs::Twist output = watchdog.CommandFor(now, &fresh);

  EXPECT_FALSE(fresh);
  EXPECT_DOUBLE_EQ(0.0, output.linear.x);
  EXPECT_DOUBLE_EQ(0.0, output.linear.y);
  EXPECT_DOUBLE_EQ(0.0, output.angular.z);
}

TEST(CmdVelWatchdogTest, ExpiresCommandAfterConfiguredTimeout)
{
  CmdVelWatchdog watchdog;
  watchdog.SetTimeout(0.20);
  const auto received = CmdVelWatchdog::TimePoint(Milliseconds(1000));
  geometry_msgs::Twist command;
  command.linear.x = 0.2;
  command.angular.z = -0.4;
  watchdog.Update(command, received);

  bool fresh = false;
  geometry_msgs::Twist output =
      watchdog.CommandFor(received + Milliseconds(200), &fresh);
  EXPECT_TRUE(fresh);
  EXPECT_DOUBLE_EQ(0.2, output.linear.x);
  EXPECT_DOUBLE_EQ(-0.4, output.angular.z);

  output = watchdog.CommandFor(received + Milliseconds(201), &fresh);
  EXPECT_FALSE(fresh);
  EXPECT_DOUBLE_EQ(0.0, output.linear.x);
  EXPECT_DOUBLE_EQ(0.0, output.angular.z);
}

TEST(CmdVelWatchdogTest, LockImmediatelyRevokesCommand)
{
  CmdVelWatchdog watchdog;
  const auto now = CmdVelWatchdog::TimePoint(Milliseconds(1000));
  geometry_msgs::Twist command;
  command.linear.x = 0.1;
  watchdog.Update(command, now);
  watchdog.Lock();

  bool fresh = true;
  const geometry_msgs::Twist output = watchdog.CommandFor(now, &fresh);
  EXPECT_FALSE(fresh);
  EXPECT_DOUBLE_EQ(0.0, output.linear.x);
}

TEST(CmdVelWatchdogTest, FinalLimiterClampsDifferentialDriveCommand)
{
  geometry_msgs::Twist command;
  command.linear.x = -0.8;
  command.linear.y = 0.3;
  command.linear.z = 1.0;
  command.angular.x = 1.0;
  command.angular.y = 1.0;
  command.angular.z = 0.9;

  const geometry_msgs::Twist limited = LimitCmdVel(command, 0.20, 0.0, 0.40);

  EXPECT_DOUBLE_EQ(-0.20, limited.linear.x);
  EXPECT_DOUBLE_EQ(0.0, limited.linear.y);
  EXPECT_DOUBLE_EQ(0.0, limited.linear.z);
  EXPECT_DOUBLE_EQ(0.0, limited.angular.x);
  EXPECT_DOUBLE_EQ(0.0, limited.angular.y);
  EXPECT_DOUBLE_EQ(0.40, limited.angular.z);
}

TEST(DriverSafetyParameterTest, RejectsNonFiniteValues)
{
  using wheeltec_driver_safety::ClampFiniteParameter;
  const double fallback = 0.20;

  EXPECT_DOUBLE_EQ(fallback,
                   ClampFiniteParameter(
                       std::numeric_limits<double>::quiet_NaN(), fallback,
                       0.05, 0.25));
  EXPECT_DOUBLE_EQ(fallback,
                   ClampFiniteParameter(
                       std::numeric_limits<double>::infinity(), fallback,
                       0.05, 0.25));
  EXPECT_DOUBLE_EQ(fallback,
                   ClampFiniteParameter(
                       -std::numeric_limits<double>::infinity(), fallback,
                       0.05, 0.25));
}

TEST(DriverSafetyParameterTest, EnforcesHardBounds)
{
  using wheeltec_driver_safety::ClampFiniteParameter;

  EXPECT_DOUBLE_EQ(0.05,
                   ClampFiniteParameter(-1.0, 0.20, 0.05, 0.25));
  EXPECT_DOUBLE_EQ(0.25,
                   ClampFiniteParameter(10.0, 0.20, 0.05, 0.25));
  EXPECT_DOUBLE_EQ(20.0,
                   ClampFiniteParameter(1.0, 20.0, 20.0, 100.0));
  EXPECT_DOUBLE_EQ(100.0,
                   ClampFiniteParameter(1000.0, 20.0, 20.0, 100.0));
  EXPECT_DOUBLE_EQ(0.20,
                   ClampFiniteParameter(1.0, 0.20, 0.0, 0.20));
  EXPECT_DOUBLE_EQ(0.40,
                   ClampFiniteParameter(1.0, 0.40, 0.0, 0.40));
  EXPECT_EQ(10,
            wheeltec_driver_safety::ClampIntegerParameter(100, 1, 10));
}

TEST(DriverPublisherIdentityTest, AcceptsOnlySafetyGateCaller)
{
  ros::M_string connection_header;
  connection_header["callerid"] = "/wheeltec_safety";

  EXPECT_TRUE(
      wheeltec_driver_safety::IsExpectedCmdVelPublisher(connection_header));
  EXPECT_STREQ("/wheeltec_driver/cmd_vel",
               wheeltec_driver_safety::kDriverCmdVelTopic);
  EXPECT_STREQ("/wheeltec_safety",
               wheeltec_driver_safety::kExpectedCmdVelCallerId);
  EXPECT_TRUE(wheeltec_driver_safety::IsInvariantDriverTopic(
      "/wheeltec_driver/cmd_vel"));
}

TEST(DriverPublisherIdentityTest, RejectsUnexpectedOrMissingCaller)
{
  ros::M_string connection_header;
  connection_header["callerid"] = "/move_base";
  EXPECT_FALSE(
      wheeltec_driver_safety::IsExpectedCmdVelPublisher(connection_header));

  connection_header["callerid"] = "/teleop_twist_keyboard";
  EXPECT_FALSE(
      wheeltec_driver_safety::IsExpectedCmdVelPublisher(connection_header));

  connection_header.erase("callerid");
  EXPECT_FALSE(
      wheeltec_driver_safety::IsExpectedCmdVelPublisher(connection_header));

  EXPECT_FALSE(wheeltec_driver_safety::IsInvariantDriverTopic("/cmd_vel"));
  EXPECT_FALSE(wheeltec_driver_safety::IsInvariantDriverTopic(
      "/another_namespace/wheeltec_driver/cmd_vel"));
}

TEST(DriverPublisherIdentityTest, SourceFaultRemainsLatchedForProcessLifetime)
{
  wheeltec_driver_safety::CmdVelSourceGuard guard;
  ros::M_string expected_header;
  expected_header["callerid"] = "/wheeltec_safety";
  ros::M_string unexpected_header;
  unexpected_header["callerid"] = "/move_base";

  EXPECT_TRUE(guard.Accept(&expected_header));
  EXPECT_FALSE(guard.IsFaultLatched());
  EXPECT_FALSE(guard.Accept(&unexpected_header));
  EXPECT_TRUE(guard.IsFaultLatched());
  EXPECT_FALSE(guard.Accept(&expected_header));
  EXPECT_TRUE(guard.IsFaultLatched());
}

TEST(DriverPublisherIdentityTest, MissingConnectionHeaderLatchesFault)
{
  wheeltec_driver_safety::CmdVelSourceGuard guard;
  ros::M_string expected_header;
  expected_header["callerid"] = "/wheeltec_safety";

  EXPECT_FALSE(guard.Accept(nullptr));
  EXPECT_TRUE(guard.IsFaultLatched());
  EXPECT_FALSE(guard.Accept(&expected_header));
}

}  // namespace
