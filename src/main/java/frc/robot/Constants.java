// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.lib.W8.util.Device;
import frc.lib.W8.util.Device.CAN;
import frc.lib.W8.util.MechanismUtil.DistanceAngleConverter;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static final String CanIDs = null;

  public static final boolean tuningMode = false;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public class FieldConstants {
    /**
     *     Contains various field dimensions and useful reference points. All units are in meters
     * and poses    have a blue alliance origin.
     */
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public static final Distance FIELDLENGTH = Meters.of(aprilTagLayout.getFieldLength());
    public static final Distance FIELDWIDTH = Meters.of(aprilTagLayout.getFieldWidth());
    public static final Distance STARTINGLINEX = Inches.of(299.438);
    public static final Translation2d FIELDCENTER =
        new Translation2d(FIELDLENGTH.in(Meters) / 2, FIELDWIDTH.in(Meters) / 2);
    public static final Distance ALGAEDIAMETER = Meters.of(.41);
  }

  public class HopperConstants {
    // holds constants for the hopper
    public static final Distance TOLERANCE = Inches.of(2.0);
    public static final Double GEARING = (5.0 / 1.0);
    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(15.0);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);

    // CHANGE TO PROPER RPMS !!!!
    public static final double SLOW_SPEED_RPM = 0.0;
    public static final double FAST_SPEED_RPM = 0.0;
    public static final double REVERSE_SPEED_RPM = 0.0;
    public static final Voltage VOLTAGE = Volts.of(12.0);
    private static final Distance DRUM_RADIUS = Inches.of(2.0);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);
    public static final AngularVelocity ANGULAR_VELOCITY = RotationsPerSecond.of(1);
    public static final AngularAcceleration ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1);
  }

  public class Ports {
    // Constants for Port Values !!!!!!
    public static final Device.CAN IntakeRoller = new CAN(1, "rio");
    public static final Device.CAN LEDs = new CAN(2, "rio");
    public static final Device.CAN HopperRoller = new CAN(3, "rio");
    public static final Device.CAN ClimberLinearMechanism = new CAN(4, "rio");
  }

  public final class ShooterConstants {
    // Constants for the Shooter
    public static final Angle ANGLE_TOLERANCE = Rotations.of(0.01);
    public static final AngularVelocity ANGLE_VELOCITY_TOLERANCE = RotationsPerSecond.of(0.01);
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(204);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(204);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    public static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
    public static final double SENSOR_TO_MECHANISM = (204.0 / 1.0);
    public static final Translation3d OFFSET = Translation3d.kZero;
    public static final Angle MIN_ANGLE = Rotations.of(0.0);
    public static final Angle MAX_ANGLE = Rotations.of(10.0);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final Distance WHEEL_RADIUS = Meters.of(0.5);
    public static final double IDLE_SPEED_RPM = (1.0);
    public static final double HUB_SPEED_RPM = (1.0);
    public static final double TOWER_SPEED_RPM = (1.0);
    public static final double DEFAULT_SPEED_RPM = (1.0);
    public static final double FLYWHEEL_VELOCITY_TOLERANCE = 1.0;
    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);
  }

  public static final int CANDLE_ID = 50;

  public class IntakeConstants {
    // constants the intake :thumbs_up: :D
    public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(0.0);
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(0.0);
    public static final AngularAcceleration ACCELERATION = RotationsPerSecondPerSecond.of(0.0);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    public static final Angle MIN_ANGLE = Rotations.of(0.0);
    public static final Angle MAX_ANGLE = Rotations.of(1);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final Distance WHEEL_RADIUS = Meters.of(0.05);
    public static final Translation3d OFFSET = Translation3d.kZero;
    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);
  }

  public class FeederConstants {
    public static final AngularVelocity FEED_SPEED = RotationsPerSecond.of(0.0);
    public static final AngularAcceleration FEED_ACCELERATION = RotationsPerSecondPerSecond.of(0.0);
  }

  public class ClimberConstants {
    public static final Distance TOLERANCE = Inches.of(0.1);
    public static final double GEARING = (5.0 / 1.0);
    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(10.0);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);
    public static final Distance DRUM_RADIUS = Inches.of(2.0);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);
    public static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(
            new Translation3d(0.0, 0.0, 0.0),
            MIN_DISTANCE,
            MAX_DISTANCE,
            STARTING_DISTANCE,
            CONVERTER);
    public static final double CLIMB_SPEED = 1.0;
  }
}
