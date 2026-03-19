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

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Foot;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.signals.Enable5VRailValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
// import edu.wpi.first.wpilibj.DigitalInput;
// import edu.wpi.first.wpilibj.DutyCycle;
// import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.lib.W8.io.motor.MotorIOTalonFX.TalonFXFollower;
import frc.lib.W8.mechanisms.linear.LinearMechanism.LinearMechCharacteristics;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism.RotaryMechCharacteristics;
import frc.lib.W8.util.Device;
import frc.lib.W8.util.Device.CAN;
import frc.lib.W8.util.MechanismUtil.DistanceAngleConverter;
import java.util.Arrays;
import java.util.List;
import org.photonvision.simulation.VisionSystemSim;

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

  public class LEDConstants {
    public static final RainbowAnimation rainbowAnim = new RainbowAnimation(0, 2);
    public static final RGBWColor colorPaleBlue = new RGBWColor(165, 180, 208, 0);
    public static final RGBWColor colorWheezerBlue = new RGBWColor(24, 155, 204, 0);
    public static final CANdleConfiguration CANDLE_CONFIG =
        new CANdleConfiguration()
            .withCANdleFeatures(
                new CANdleFeaturesConfigs()
                    .withEnable5VRail(Enable5VRailValue.Enabled)
                    .withVBatOutputMode(VBatOutputModeValue.On)
                    .withStatusLedWhenActive(StatusLedWhenActiveValue.Disabled))
            .withLED(
                new LEDConfigs()
                    .withBrightnessScalar(1.0)
                    .withStripType(StripTypeValue.RGB)
                    .withLossOfSignalBehavior(LossOfSignalBehaviorValue.DisableLEDs));
  }

  public class FieldConstants {
    /**
     * Contains various field dimensions and useful reference points. All units are in meters and
     * poses have a blue alliance origin.
     */
    public static final AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

    public static final double HUBWIDTH = Units.inchesToMeters(47.0);
    public static final Distance FIELDLENGTH = Meters.of(aprilTagLayout.getFieldLength());
    public static final Distance FIELDWIDTH = Meters.of(aprilTagLayout.getFieldWidth());
    public static final Distance STARTINGLINEX = Inches.of(299.438);
    public static final Translation2d FIELDCENTER =
        new Translation2d(FIELDLENGTH.in(Meters) / 2, FIELDWIDTH.in(Meters) / 2);
    public static final Distance CENTERLINE = Meters.of(FIELDLENGTH.in(Meters) / 2);
    public static final Double starting = (aprilTagLayout.getTagPose(26).get().getX());
    public static final Double allianceZone = starting;
    public static final Double hubCenter =
        (aprilTagLayout.getTagPose(26).get().getX() + (HUBWIDTH / 2.0));
    public static final Distance neutralZoneNear =
        Meters.of(CENTERLINE.in(Meters) - (Units.inchesToMeters(120.0)));
    public static final Distance neutralZoneFar =
        Meters.of(CENTERLINE.in(Meters) + (Units.inchesToMeters(120.0)));
    public static final Distance oppHubCenter =
        Meters.of(aprilTagLayout.getTagPose(4).get().getX() + (HUBWIDTH / 2.0));
    public static final Distance oppAllianceZone =
        Meters.of(aprilTagLayout.getTagPose(10).get().getX());
    public static final double BUMPWIDTH = Units.inchesToMeters(73.0);
    public static final double BUMPHIGHT = Units.inchesToMeters(6.513);
    public static final double BUMPDEPTH = Units.inchesToMeters(44.4);
    public static final double RBUMPSTART = hubCenter + (HUBWIDTH / 2.0);
    public static final double RBUMPEND = RBUMPSTART + BUMPWIDTH;
    public static final double LBUMPSTART = hubCenter - (HUBWIDTH / 2.0);
    public static final double LBUMPEND = RBUMPSTART - BUMPWIDTH;
  }

  public class Ports {
    // Constants for Port Values
    public static final Device.CAN IntakeRoller = new CAN(11, "rio");
    public static final Device.CAN IntakePivot = new CAN(22, "rio");

    public static final Device.CAN Spindexer = new CAN(40, "rio");

    public static final Device.CAN TowerRoller = new CAN(13, "rio");
    public static final Device.CAN LeftFlywheel = new CAN(43, "rio");
    public static final Device.CAN RightFlywheel = new CAN(56, "rio");
    public static final Device.CAN HoodMotor = new CAN(44, "rio");

    public static final Device.CAN ClimberMotor = new CAN(50, "rio");

    public static final Device.CAN LEDs = new CAN(2, "rio");
    public static final Device.CAN Pigeon = new CAN(0, "rio");
  }

  public class HopperConstants {
    // holds constants for the hopper

    public static final String MOTOR_NAME = "Spindexer";

    // CHANGE TO PROPER RPMS !!!!

    public static final double SLOW_SPEED_RPM = 0.0;
    public static final double FAST_SPEED_RPM = 100.0;
    public static final Voltage VOLTAGE = Volts.of(12.0);
    public static final AngularVelocity ANGULAR_VELOCITY = RotationsPerSecond.of(1);
    public static final AngularAcceleration ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1);
    public static final int HOPPER_POSITION = 1;

    public static final Mass CARRIAGE_MASS = Kilograms.of(2.5);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0028125);

    // Mechanism Constants
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(3200 / 60);
    public static final AngularAcceleration MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(3200 / 60 * 10);
    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI).times(10.0);
    public static final AngularAcceleration ACCELERATION = CRUISE_VELOCITY.div(0.1).per(Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    public static final double GEARING = (1.0 / 1.0);
    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(10.0);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);

    public static final Distance DRUM_RADIUS = Inches.of(2.0);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    public static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(
            new Translation3d(0.0, 0.0, 0.0),
            MIN_DISTANCE,
            MAX_DISTANCE,
            STARTING_DISTANCE,
            CONVERTER);

    public static TalonFXConfiguration getFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.StatorCurrentLimit = 80.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      //     CONVERTER.toAngle(MAX_DISTANCE).in(Rotations);

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      //     CONVERTER.toAngle(MIN_DISTANCE).in(Rotations);

      config.Feedback.RotorToSensorRatio = 1.0;

      config.Feedback.SensorToMechanismRatio = GEARING;

      if (RobotBase.isReal()) {
        config.Slot0 =
            new Slot0Configs().withKP(10).withKI(0.0).withKD(0.0).withKS(2.5).withKV(0.05);

      } else {
        config.Slot0 = new Slot0Configs().withKP(0.75).withKI(0.0).withKD(0.0);
      }

      config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
      config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
      config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

      return config;
    } // End here
  }

  public final class ShooterConstants {
    // Constants for Shooter
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

    // Hood Constants
    public static final double HEIGHT_DIFFERENCE =
        1.295; // Meters between flywheel center and top of hub opening
    public static final double EXIT_VELOCITY = 7.4; // m/s from ReCalc Flywheel Calculator
    public static final AngularVelocity HOOD_VELOCITY = RotationsPerSecond.of(-0.0005);
    public static final AngularAcceleration HOOD_ACCELERATION = RotationsPerSecondPerSecond.of(.2);
    public static final Velocity<AngularAccelerationUnit> HOOD_JERK = HOOD_ACCELERATION.per(Second);
    public static final double HOOD_TOLERANCE = 1.0; // In degrees
    public static final double GRAVITY = 9.81; // m/s^2
    public static final double IDLE_HOOD_ANGLE = 25.0; // degrees
    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final double HARD_STOP_CURRENT_LIMIT = 3.0;

    public static final double TOWER_ANGLE = 8.9; // degrees // Distance 120.5 Inches
    public static final double HUB_ANGLE = 2.3; // degrees // Distance 49.5 inches
    public static final double TRENCH_ANGLE = 9.8; // degrees

    public static final double ANGLE_AT_96 = 6.8; // degrees // Distance 96 inches
    public static final double ANGLE_AT_152 = 12.1; // degrees  // Distance 152 inches
    public static final double ANGLE_AT_197 = 18.6; // degrees // Distance
  }

  public class ShooterFlywheelConstants {

    public static String NAME = "ShooterFlywheel";

    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second);
    public static final AngularVelocity IDLE_SPEED = RotationsPerSecond.of(30);

    private static final double GEARING = (1.0 / 1.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.0);

    public static final TalonFXFollower FOLLOWER_1 =
        new TalonFXFollower(Ports.LeftFlywheel, MotorAlignmentValue.Opposed);

    public static TalonFXConfiguration getFXConfig(boolean invert) {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.StatorCurrentLimit = 80.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      if (invert == true) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      } else {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      }

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

      config.Feedback.RotorToSensorRatio = 1.0;

      config.Feedback.SensorToMechanismRatio = GEARING;

      if (RobotBase.isReal()) {
        // Velocity PID
        config.Slot0 =
            new Slot0Configs().withKP(3.5).withKI(0.15).withKD(0.0).withKV(0.1).withKS(8.5);
      } else {
        config.Slot0 = new Slot0Configs().withKP(0.75).withKI(0.0).withKD(0.0);
      }

      return config;
    }
  }

  public class ShooterRotaryConstants {
    public static String NAME = "ShooterRotary";

    public static final Angle TOLERANCE = Degrees.of(2.0);

    public static final AngularVelocity CRUISE_VELOCITY = RadiansPerSecond.of(100);
    public static final AngularAcceleration ACCELERATION = RadiansPerSecondPerSecond.of(200);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = 1 / 50; // 1.0

    public static final Translation3d OFFSET = Translation3d.kZero;

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(25);
    public static final Angle STARTING_ANGLE = Degrees.of(0.0);
    public static final Distance ARM_LENGTH = Meters.of(0.0); // 0.5

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final Mass ARM_MASS = Kilograms.of(.01);
    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI =
        KilogramSquareMeters.of(
            SingleJointedArmSim.estimateMOI(ARM_LENGTH.in(Meters), ARM_MASS.in(Kilograms)));

    private static final Angle ENCODER_OFFSET = Rotations.of(0.0);

    // // Positional PID
    // private static Slot0Configs SLOT0CONFIG =
    //     new Slot0Configs().withKP(500.0).withKI(0.0).withKD(0.0).withKS(10.0).withKV(0.0);
    // // Velocity PID
    // private static Slot1Configs SLOT1CONFIG =
    //     new Slot1Configs().withKP(30.0).withKI(0.0).withKD(0.0).withKS(40.0).withKV(0.0);

    /**
     * Creates and returns the TalonFX motor controller configuration for the rotary mechanism.
     *
     * <p>This configuration includes:
     *
     * <ul>
     *   <li>Current limits to prevent motor damage and brownouts
     *   <li>Voltage limits for power output
     *   <li>Brake mode to hold position when not moving
     *   <li>Software limit switches to prevent mechanism damage
     *   <li>Gear ratios for proper position/velocity feedback
     *   <li>Remote CANcoder feedback for absolute positioning
     *   <li>PID gains for control
     * </ul>
     *
     * @return A configured TalonFXConfiguration object ready to apply to a motor controller
     */
    public static TalonFXConfiguration getFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = false;
      config.CurrentLimits.StatorCurrentLimit = 120.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Degrees);

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Degrees);

      config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR;

      if (RobotBase.isReal()) {
        config.Feedback.SensorToMechanismRatio = 1 / 50;
        // Positional PID
        config.Slot0 =
            new Slot0Configs().withKP(500.0).withKI(0.0).withKD(0.0).withKS(10.0).withKV(0.0);
        // Velocity PID
        config.Slot1 =
            new Slot1Configs().withKP(30.0).withKI(0.0).withKD(0.0).withKS(40.0).withKV(0.0);

      } else {
        config.Feedback.SensorToMechanismRatio = (50.0 / 1.0);
        config.Slot0 = new Slot0Configs().withKP(30.0).withKI(0.0).withKD(5.0);
      }

      config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
      config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);

      return config;
    }

    /**
     * Creates and returns the CANcoder absolute encoder configuration.
     *
     * <p>The CANcoder provides absolute position feedback, meaning it knows the true position of
     * the mechanism even after power cycling. The magnet offset calibrates the encoder's zero
     * position.
     *
     * @param sim Whether this configuration is for simulation (true) or real robot (false). In
     *     simulation, the offset is set to 0.0 since it's not needed.
     * @return A configured CANcoderConfiguration object
     */
    public static CANcoderConfiguration getCANcoderConfig(boolean sim) {
      CANcoderConfiguration config = new CANcoderConfiguration();

      config.MagnetSensor.MagnetOffset = sim ? 0.0 : ENCODER_OFFSET.in(Rotations);

      return config;
    }
  }

  // // Needs tuned
  // public class ShooterTowerConstants {
  //   public static String NAME = "ShooterTowerFlywheel";

  //   public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
  //   public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second);

  //   private static final double GEARING = (2.0 / 1.0);

  //   public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

  //   public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
  //   public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.0);

  //   // Velocity PID
  //   private static Slot0Configs SLOT0CONFIG =
  //       new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0);

  //   public static TalonFXConfiguration getFXConfig() {
  //     TalonFXConfiguration config = new TalonFXConfiguration();

  //     config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
  //     config.CurrentLimits.SupplyCurrentLimit = 40.0;
  //     config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
  //     config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

  //     config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
  //     config.CurrentLimits.StatorCurrentLimit = 80.0;

  //     config.Voltage.PeakForwardVoltage = 12.0;
  //     config.Voltage.PeakReverseVoltage = -12.0;

  //     config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
  //     config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

  //     config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

  //     config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

  //     config.Feedback.RotorToSensorRatio = 1.0;

  //     config.Feedback.SensorToMechanismRatio = GEARING;

  //     config.Slot0 = SLOT0CONFIG;

  //     return config;
  //   }
  // }

  public static final int CANDLE_ID = 50;

  public class IntakeFlywheelConstants {
    // Constants for Intake
    public static final Angle MIN_ANGLE = Rotations.of(0.0);
    public static final Angle MAX_ANGLE = Rotations.of(1);
    public static final Angle STARTING_ANGLE = Rotations.of(0.0);
    public static final double PICKUP_SPEED = 30.0;
    public static final Distance WHEEL_RADIUS = Meters.of(0.05);
    public static final Translation3d OFFSET = Translation3d.kZero;
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.0028125);
    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(OFFSET, WHEEL_RADIUS, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);
    public static final String MOTOR_NAME = "Intake Flywheel";

    // Mechanism Constants
    public static final AngularVelocity MAX_VELOCITY = RotationsPerSecond.of(3200 / 60);
    public static final AngularAcceleration MAX_ACCELERATION =
        RotationsPerSecondPerSecond.of(3200 / 60 * 10);
    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI).times(10.0);
    public static final AngularAcceleration ACCELERATION = CRUISE_VELOCITY.div(0.1).per(Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    public static final double GEARING = (22.0 / 18.0);
    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(10.0);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);

    public static final Distance DRUM_RADIUS = Inches.of(2.0);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);
    private static final Angle ENCODER_OFFSET = Rotations.of(0.0);

    public static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(
            new Translation3d(0.0, 0.0, 0.0),
            MIN_DISTANCE,
            MAX_DISTANCE,
            STARTING_DISTANCE,
            CONVERTER);

    public static CANcoderConfiguration getCANcoderConfig(boolean sim) {
      CANcoderConfiguration config = new CANcoderConfiguration();

      config.MagnetSensor.MagnetOffset = sim ? 0.0 : ENCODER_OFFSET.in(Rotations);

      return config;
    }

    // config.Slot0 = new Slot0Configs().withKP(0.75).withKI(0.0).withKD(0.0);
    // private static Slot0Configs SLOT0CONFIG =
    //     new Slot0Configs().withKP(10).withKI(0.0).withKD(0.0).withKV(0.05).withKS(8.0);

    public static TalonFXConfiguration getFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.StatorCurrentLimit = 80.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      //     CONVERTER.toAngle(MAX_DISTANCE).in(Rotations);

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      //     CONVERTER.toAngle(MIN_DISTANCE).in(Rotations);

      config.Feedback.RotorToSensorRatio = 1.0;

      config.Feedback.SensorToMechanismRatio = GEARING;

      if (RobotBase.isReal()) {
        config.Slot0 =
            new Slot0Configs().withKP(10).withKI(0.0).withKD(0.0).withKV(0.05).withKS(8.0);
      } else {
        config.Slot0 = new Slot0Configs().withKP(0.75).withKI(0.0).withKD(0.0);
      }

      config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
      config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
      config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

      return config;
    }
  }

  public class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera names, must match names configured on coprocessor
    public static String camera0Name = "camera_0";
    public static String camera1Name = "camera_1";

    // Robot to camera transforms
    // (Not used by Limelight, configure in web UI instead)
    public static Transform3d robotToCamera0 =
        new Transform3d(0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, 0.0));
    public static Transform3d robotToCamera1 =
        new Transform3d(-0.2, 0.0, 0.2, new Rotation3d(0.0, -0.4, Math.PI));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    /** Tags used for reef alignment */
    public static List<Integer> alignmentTags =
        Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    public static VisionSystemSim getSystemSim() {
      var system = new VisionSystemSim("main");
      system.addAprilTags(aprilTagLayout);
      return system;
    }
  }

  public class IntakePivotConstants {
    public static final String NAME = "Intake";

    public static final Angle PICKUP_ANGLE = Degrees.of(123.0); // 123.0
    public static final Angle STOW_ANGLE = Degrees.of(0.0);
    public static final Angle JOSTLE_ANGLE = Degrees.of(40.0);

    public static final Angle TOLERANCE = Degrees.of(3.0);

    public static final AngularVelocity CRUISE_VELOCITY = RadiansPerSecond.of(100);
    public static final AngularAcceleration ACCELERATION = RadiansPerSecondPerSecond.of(200);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);

    private static final double ROTOR_TO_SENSOR = (1.0 / 1.0);
    private static final double SENSOR_TO_MECHANISM = 17.35;

    public static final Angle MIN_ANGLE = Degrees.of(0.0);
    public static final Angle MAX_ANGLE = Degrees.of(126.0);
    public static final Angle STARTING_ANGLE = Degrees.of(0.0);
    public static final Distance ARM_LENGTH = Foot.one();

    public static final RotaryMechCharacteristics CONSTANTS =
        new RotaryMechCharacteristics(
            new Translation3d(), ARM_LENGTH, MIN_ANGLE, MAX_ANGLE, STARTING_ANGLE);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(0.25);

    // public static final int ENCODER_CHANNEL = 9;
    // public static final DutyCycleEncoder ENCODER1 =
    //     new DutyCycleEncoder(new DutyCycle(new DigitalInput(ENCODER_CHANNEL)));

    // public static final Encoder ENCODER = new Encoder(ENCODER_CHANNEL, ENCODER_CHANNEL);

    // Positional PID

    // .withKG(15)
    // .withGravityType(GravityTypeValue.Arm_Cosine);

    public static TalonFXConfiguration getFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = false;
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = false;
      config.CurrentLimits.StatorCurrentLimit = 120.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = MAX_ANGLE.in(Rotations);

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = MIN_ANGLE.in(Rotations);

      config.Feedback.RotorToSensorRatio = ROTOR_TO_SENSOR;
      config.Feedback.SensorToMechanismRatio = SENSOR_TO_MECHANISM;

      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      if (RobotBase.isReal()) {
        config.Slot0 =
            new Slot0Configs().withKP(300.0).withKI(1000).withKD(80).withKS(8.0).withKV(0.0);

      } else {
        config.Slot0 =
            new Slot0Configs().withKP(100.0).withKI(0.0).withKD(0).withKS(0.07).withKV(0.1);
      }
      config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
      config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
      config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

      return config;
    }
  }

  public class FeederConstants {
    public static final AngularVelocity FEED_SPEED = RotationsPerSecond.of(20.0);
    public static final AngularAcceleration FEED_ACCELERATION = RotationsPerSecondPerSecond.of(50);

    public static String NAME = "ShooterTower";

    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration MAX_ACCELERATION = MAX_VELOCITY.per(Second);

    private static final double GEARING = (15.0 / 7.0);

    public static final AngularVelocity TOLERANCE = MAX_VELOCITY.times(0.1);

    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final MomentOfInertia MOI = KilogramSquareMeters.of(1.0);

    // Velocity PID
    // private static Slot0Configs SLOT0CONFIG =
    //     new Slot0Configs().withKP(5.0).withKI(0.0).withKD(0.0).withKV(0.05).withKS(12.0);

    public static TalonFXConfiguration getFXConfig(boolean invert) {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.StatorCurrentLimit = 80.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      // config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      if (invert == true) {
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
      } else {
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      }

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;

      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

      config.Feedback.RotorToSensorRatio = 1.0;

      config.Feedback.SensorToMechanismRatio = GEARING;

      // config.Slot0 = SLOT0CONFIG;
      if (RobotBase.isReal()) {
        config.Slot0 =
            new Slot0Configs().withKP(5.0).withKI(0.0).withKD(0.0).withKV(0.05).withKS(12.0);

      } else {
        config.Slot0 = new Slot0Configs().withKP(0.0).withKI(0.0).withKD(0.0).withKS(10.0);
      }

      return config;
    }
  }

  public class ClimberConstants {
    public static final DCMotor DCMOTOR = DCMotor.getKrakenX60(1);
    public static final Mass CARRIAGE_MASS = Kilograms.of(2.5);
    public static final String MOTOR_NAME = "Climber motor";
    public static final String NAME = "Climber";
    public static final Distance TOLERANCE = Inches.of(0.1);
    public static final double GEARING = 140.8;
    public static final Distance MIN_DISTANCE = Inches.of(0.0);
    public static final Distance MAX_DISTANCE = Inches.of(10.0);
    public static final Distance STARTING_DISTANCE = Inches.of(0.0);
    public static final Distance DRUM_RADIUS = Inches.of(2.0);
    public static final DistanceAngleConverter CONVERTER = new DistanceAngleConverter(DRUM_RADIUS);

    public static final AngularVelocity CRUISE_VELOCITY =
        RadiansPerSecond.of(2 * Math.PI).times(800.0 / GEARING);
    public static final AngularVelocity LOWER_VELOCITY =
        RadiansPerSecond.of(-2 * Math.PI).times(800.0 / GEARING);
    public static final AngularVelocity CALIBRATE_VELOCITY =
        RadiansPerSecond.of(-2 * Math.PI).times(100.0 / GEARING);

    public static final AngularAcceleration ACCELERATION = CRUISE_VELOCITY.div(0.1).per(Second);
    public static final Velocity<AngularAccelerationUnit> JERK = ACCELERATION.per(Second);
    public static final Distance BOTTOM = Inches.of(0.0);
    public static final Distance MIDDLE = Inches.of(15.0);
    public static final Distance TOP = Inches.of(30.0);

    public static final double L1 = 10;
    public static final double STOW = 0;

    public static final LinearMechCharacteristics CHARACTERISTICS =
        new LinearMechCharacteristics(
            new Translation3d(0.0, 0.0, 0.0),
            MIN_DISTANCE,
            MAX_DISTANCE,
            STARTING_DISTANCE,
            CONVERTER);

    public static TalonFXConfiguration getFXConfig() {
      TalonFXConfiguration config = new TalonFXConfiguration();

      config.CurrentLimits.SupplyCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.SupplyCurrentLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerLimit = 40.0;
      config.CurrentLimits.SupplyCurrentLowerTime = 0.1;

      config.CurrentLimits.StatorCurrentLimitEnable = Robot.isReal();
      config.CurrentLimits.StatorCurrentLimit = 80.0;

      config.Voltage.PeakForwardVoltage = 12.0;
      config.Voltage.PeakReverseVoltage = -12.0;

      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

      config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
      // config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
      //     CONVERTER.toAngle(MAX_DISTANCE).in(Rotations);
      config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
      // config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
      // config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
      //     CONVERTER.toAngle(MIN_DISTANCE).in(Rotations);

      config.Feedback.RotorToSensorRatio = 1.0;

      config.Feedback.SensorToMechanismRatio = GEARING;

      if (Robot.isReal()) {
        config.Slot0 = new Slot0Configs().withKP(10).withKI(0.0).withKD(0.0);
        config.Slot1 = new Slot1Configs().withKP(1).withKI(0.0).withKD(0.0);
      } else {
        config.Slot0 = new Slot0Configs().withKP(0.75).withKI(0.0).withKD(0.0);
      }

      config.MotionMagic.MotionMagicCruiseVelocity = CRUISE_VELOCITY.in(RotationsPerSecond);
      config.MotionMagic.MotionMagicAcceleration = ACCELERATION.in(RotationsPerSecondPerSecond);
      config.MotionMagic.MotionMagicJerk = JERK.in(RotationsPerSecondPerSecond.per(Second));

      return config;
    } // End here

    public static final Distance ELEVATOR_RADIUS = Inches.of(2.0);
    public static final AngularVelocity ANGULAR_VELOCITY = RotationsPerSecond.of(1);
    public static final AngularAcceleration ANGULAR_ACCELERATION =
        RotationsPerSecondPerSecond.of(1);
    public static final double CLIMB_SPEED = 1.0;
    public static final double HARD_STOP_CURRENT_LIMIT = 20.0;
  }
}
