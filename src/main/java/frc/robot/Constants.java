// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeConstants {
    // TODO set the correct CAN ID for the intake motor
    public static final int kIntakeMotorCANID = 8;

    public static final double motorReduction = 1.0; // Gear reduction from motor to intake wheel
    public static final double currentLimit = 30.0; // Current limit in Amps
    public static final double realKp = 0.1; // Proportional gain for the PID controller
    public static final double realKd = 0.0; // Derivative gain for the PID controller

    public static final boolean Inverted = false; // Inversion for the intake motor
  }
}
