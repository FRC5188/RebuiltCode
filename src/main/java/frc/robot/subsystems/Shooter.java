package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterRotaryConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import frc.lib.firecontrol.ShotCalculator;
import frc.lib.firecontrol.ShotLUT;

public class Shooter extends SubsystemBase {

  private final FlywheelMechanism _flywheel;
  private final FlywheelMechanism _feeder;
  private final RotaryMechanism _hood;

  // desired target values
  private double desiredVelo;
  private double hoodAngle;
  private double desiredHoodAngle;


   private static boolean autoShootEnabled = true;

  public AngularVelocity targetVelocity = RotationsPerSecond.of(0.0);
  public AngularVelocity feederTargetVelocity = RotationsPerSecond.of(0.0);

  private Debouncer homeDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private Trigger homedTrigger;

  private final ShotCalculator shotCalculator; // NEW!

  public Shooter(FlywheelMechanism rflywheel, FlywheelMechanism feeder, RotaryMechanism hood) {
    _flywheel = rflywheel;
    _feeder = feeder;
    _hood = hood;
    homedTrigger =
        new Trigger(
            () ->
                homeDebouncer.calculate(
                    _hood
                        .getSupplyCurrent()
                        .gte(Amps.of(ShooterConstants.HARD_STOP_CURRENT_LIMIT))));
                        
    // Setup Shot Calculator
    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX = 0.0;  // Configure distance from robot center
    config.launcherOffsetY = 0.0;
    config.phaseDelayMs = 0.0;
    config.mechLatencyMs = 20.0;
    config.maxTiltDeg = 5.0; 
    config.headingSpeedScalar = 1.0; 
    config.headingReferenceDistance = 2.5; 
    
    shotCalculator = new ShotCalculator(config);
    
    // Option 2: ShotLUT (RPM + angle + TOF, for adjustable hoods)
      // ShotLUT lut = new ShotLUT();
      // lut.put(1.0, 2000, 45.0, 0.45);  // distance, RPM, angle, TOF
      // lut.put(2.0, 2800, 42.0, 0.62);
      // lut.put(3.0, 3500, 38.0, 0.78);
      // shotCalculator.loadShotLUT(lut);

    // Initialize your custom empirical data lookup table
    ShotLUT customLut = new ShotLUT();
    
    // Add your physical test data here:
    // format: .put(distance_meters, rpm, hood_angle_degrees, time_of_flight_seconds)
    customLut.put(1.3, 3600, 2.3, 0.11); 
    customLut.put(2.0, 3400, 6.2, 0.45);
    customLut.put(3.0, 4000, 8.8, 0.60);
    customLut.put(4.0, 4500, 12.3, 0.85);
    customLut.put(4.0, 4500, 18.6, 0.85);

    // Load custom lookup table into the shot calculator
    shotCalculator.loadShotLUT(customLut);

    // Setup tuning variables in smart dashboard
    SmartDashboard.putNumber("ShooterTuning/TuneHoodAngle", 15.0);
    SmartDashboard.putNumber("ShooterTuning/TuneRPM", 3000.0);
  }

  // Sets feeder motor speed
  public void runFeeder(AngularVelocity velocity) {
    _feeder.runVelocity(velocity, FeederConstants.FEED_ACCELERATION, PIDSlot.SLOT_0);
    feederTargetVelocity = velocity;
  }

  // Sets the flywheel velocity based on an input.
  public void setFlywheelVelocity(AngularVelocity velocity) {
    // store the desired velocity then send converted velocity to the mechanism
    // this.desiredVelo = velocity;
    // AngularVelocity angVelo = RotationsPerSecond.of(velocity);
    // AngularVelocity negangVelo = RotationsPerSecond.of(velocity);
    _flywheel.runVelocity(velocity, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
    targetVelocity = velocity;
  }

  public ShotCalculator getShotCalculator() {
    return shotCalculator;
  }

  public enum State {
    OFF(Units.RevolutionsPerSecond.of(0.0)),
    IDLE(Units.RevolutionsPerSecond.of(ShooterConstants.IDLE_SPEED_RPM / 60)),
    SHOOT_FROM_HUB(Units.RevolutionsPerSecond.of(ShooterConstants.HUB_SPEED_RPM / 60)),
    SHOOT_FROM_TOWER(Units.RevolutionsPerSecond.of(ShooterConstants.TOWER_SPEED_RPM / 60)),
    SHOOT(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60)),
    SHOOT_ON_MOVE(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60));

    private final AngularVelocity stateVelocity;

    State(AngularVelocity stateVelocity) {
      this.stateVelocity = stateVelocity;
    }
  }

  // Checks if the flywheel is at speed and returns a boolean
  public boolean flyAtVelocity() {
    return (Math.abs(desiredVelo - _flywheel.getVelocity().in(RotationsPerSecond)))
        <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
  }

  public double getHoodAngleDegrees(Translation2d robotPos) {

    // TODO: Replace with HUB later once it gets added.
    double distance = robotPos.getDistance(FieldConstants.FIELDCENTER);

    double check =
        Math.pow(ShooterConstants.EXIT_VELOCITY, 4)
            - ShooterConstants.GRAVITY
                * (ShooterConstants.GRAVITY * Math.pow(distance, 2)
                    + 2
                        * ShooterConstants.HEIGHT_DIFFERENCE
                        * Math.pow(ShooterConstants.EXIT_VELOCITY, 2));

    if (check < 0) {
      return ShooterConstants.IDLE_HOOD_ANGLE; // Default angle if the shot is not possible
    }
    return Math.toDegrees(
        Math.atan(
            (ShooterConstants.EXIT_VELOCITY * ShooterConstants.EXIT_VELOCITY + Math.sqrt(check))
                / (ShooterConstants.GRAVITY * distance)));
  }

  public boolean isAutoShootEnabled() {
    return autoShootEnabled;
  }

  public void setAutoShootEnabled(boolean enabled) {
    autoShootEnabled = enabled;
  }
  // Sets hood angle
  // public Command setHoodAngle(double angleDegrees) {
  //   hoodAngle = angleDegrees;
  //   desiredHoodAngle = angleDegrees;
  //   System.out.println("Setting hood angle to: " + angleDegrees + " degrees");
  //   return this.runOnce(
  //           () -> {
  //             _hood.runPosition(
  //                 Angle.ofBaseUnits(angleDegrees * ShooterConstants.SIM_MULTIPLIER, Degrees),
  //                 ShooterRotaryConstants.CRUISE_VELOCITY,
  //                 ShooterRotaryConstants.ACCELERATION,
  //                 ShooterRotaryConstants.JERK,
  //                 PIDSlot.SLOT_0);
  //           });
  // }

  // Sets hood angle returning a command
  public Command setHoodAngle(double angleDegrees) {
    return this.runOnce(() -> setHoodAngleImmediate(angleDegrees))
        .andThen(() -> System.out.println("Command finished"));
  }

  // Sets the hood angle immediately (for use inside other commands' execute loops)
  public void setHoodAngleImmediate(double angleDegrees) {
    hoodAngle = angleDegrees;
    desiredHoodAngle = angleDegrees;
    System.out.println("Setting hood angle to: " + angleDegrees + " degrees");
    _hood.runPosition(
        Angle.ofBaseUnits(angleDegrees * ShooterConstants.SIM_MULTIPLIER, Degrees),
        ShooterRotaryConstants.CRUISE_VELOCITY,
        ShooterRotaryConstants.ACCELERATION,
        ShooterRotaryConstants.JERK,
        PIDSlot.SLOT_0);
  }

  // Checks if hood is at angle
  public boolean hoodAtAngle() {
    return Math.abs(hoodAngle - _hood.getPosition().in(Degrees)) < ShooterConstants.HOOD_TOLERANCE;
  }

  // Increases Hood Angle
  public Command incrementHoodAngle() {
    Angle currentHoodAngle = _hood.getPosition().plus(Degrees.of(2.5));
    return setHoodAngle(currentHoodAngle.in(Degrees));
  }

  public Command decrementHoodAngle() {
    Angle currentHoodAngle = _hood.getPosition().minus(Degrees.of(2.5));
    return setHoodAngle(currentHoodAngle.in(Degrees));
  }

  public boolean isAboveCurrentLimit() {
    if (Math.abs(_hood.getSupplyCurrent().in(Amps)) > ShooterConstants.HARD_STOP_CURRENT_LIMIT) {
      return true;
    } else {
      return false;
    }
  }

  public Command calibrateHood() {
    return Commands.sequence(
        runOnce(() -> _hood.runVoltage(Voltage.ofBaseUnits(-1, Volts))),
        Commands.waitUntil(homedTrigger),
        runOnce(() -> _hood.setEncoderPosition(Angle.ofBaseUnits(0, Degrees))),
        runOnce(() -> _hood.runVoltage(Voltage.ofBaseUnits(0, Volts))));
  }

  public Command runFlywheel(AngularVelocity velocity) {

    return Commands.run(() -> setFlywheelVelocity(velocity), this);
  }

  public Command runTower(AngularVelocity velocity) {
    return Commands.run(() -> runFeeder(velocity), this);
  }

  public Command score() {
    return Commands.run(() -> {runFeeder(RotationsPerSecond.of(28)); setFlywheelVelocity(RotationsPerSecond.of(67));});
  }

  public Command stopScore() {
    return Commands.run(
        () -> {
          runFeeder(RotationsPerSecond.of(0));
          setFlywheelVelocity(RotationsPerSecond.of(0));
        });
  }

  public Command tuneShoot() {
    return Commands.run(
        () -> {
          double targetRpm = SmartDashboard.getNumber("ShooterTuning/TuneRPM", 3000.0);
          double targetAngle = SmartDashboard.getNumber("ShooterTuning/TuneHoodAngle", 15.0);
          
          setFlywheelVelocity(RotationsPerSecond.of(targetRpm / 60.0));
          setHoodAngleImmediate(targetAngle);
          runFeeder(RotationsPerSecond.of(60));
        }, this);
  }

  public Command readyUp() {
    return this.run(() -> {
      setFlywheelVelocity(RotationsPerSecond.of(55));
      _hood.runPosition(
          Angle.ofBaseUnits(2.3, Degrees),
          ShooterRotaryConstants.CRUISE_VELOCITY,
          ShooterRotaryConstants.ACCELERATION,
          ShooterRotaryConstants.JERK,
          PIDSlot.SLOT_0);
    });
  }

  public void simShoot() {
    if (Robot.robotContainer.intake.simBalls <= 0) return;

    Translation2d robotPose2d = Robot.robotContainer.drive.getPose().getTranslation();
    Pose3d robotPose3d =
        new Pose3d(
            new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0),
            new Rotation3d(robotPose2d.getAngle()));
    Pose3d shooterPose3d =
        new Pose3d(
            new Translation3d(-0.0075, 0.0, 0.523),
            new Rotation3d(0, _hood.getPosition().in(Radians), 0));

    double flywheelSpeed = _flywheel.getVelocity().magnitude();

    double Yaw = Robot.robotContainer.drive.getPose().getRotation().getRadians();
    double V_xy =
        Math.sin(Math.PI / 2 - (_hood.getPosition().in(Radians) + Degrees.of(12).in(Radians)))
            * flywheelSpeed;

    ChassisSpeeds driveChassisSpeeds = Robot.robotContainer.drive.getChassisSpeeds();
    Translation3d driveSpeed3d = new Translation3d(0.0, 0.0, 0.0);

    Robot.fuelSim.spawnFuel(
        robotPose3d
            .plus(
                new Transform3d(
                    shooterPose3d.getX(),
                    shooterPose3d.getY(),
                    shooterPose3d.getZ(),
                    new Rotation3d(0, 0, 0)))
            .getTranslation(),
        new Translation3d(
                V_xy * Math.cos(Yaw),
                V_xy * Math.sin(Yaw),
                Math.sin(
                        Math.PI / 2
                            - (_hood.getPosition().in(Radians) + Degrees.of(12).in(Radians)))
                    * flywheelSpeed)
            .plus(driveSpeed3d));

    Robot.robotContainer.intake.simBalls--;
  }

  private static final InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();

  static {
    hoodAngleMap.put(1.28, 2.3);
    hoodAngleMap.put(2.44, 6.8);
    hoodAngleMap.put(3.1, 8.9);
    hoodAngleMap.put(3.86, 12.1);
    hoodAngleMap.put(5.0, 18.6);
  }

  /** Distance from feed pose in meters -> flywheel speed in rotations per second */
  private static final InterpolatingDoubleTreeMap feedFlywheelMap =
      new InterpolatingDoubleTreeMap();

  static {
    feedFlywheelMap.put(0.0, 50.0);
    feedFlywheelMap.put(6.0, 50.0);
    feedFlywheelMap.put(7.0, 55.0);
    feedFlywheelMap.put(8.0, 60.0);
    feedFlywheelMap.put(20.0, 60.0);
  }

  static {
    feedFlywheelMap.put(0.0, 50.0);
    feedFlywheelMap.put(6.0, 50.0);
    feedFlywheelMap.put(7.0, 55.0);
    feedFlywheelMap.put(8.0, 60.0);
    feedFlywheelMap.put(20.0, 60.0);
  }

  public Command setAngleForDistance(DoubleSupplier distance) {
    return this.runOnce(
            () -> {
              double distanceMeters = distance.getAsDouble();
              double angle = hoodAngleMap.get(distanceMeters);
              desiredHoodAngle = angle;
              System.out.println("Setting hood angle to: " + angle + " degrees");
              _hood.runPosition(
                  Angle.ofBaseUnits(angle * ShooterConstants.SIM_MULTIPLIER, Degrees),
                  ShooterRotaryConstants.CRUISE_VELOCITY,
                  ShooterRotaryConstants.ACCELERATION,
                  ShooterRotaryConstants.JERK,
                  PIDSlot.SLOT_0);
            })
          .andThen(() -> System.out.println("Command finished"));
  }

  public void setAngleForDistance2(DoubleSupplier distance) {
    double distanceMeters = distance.getAsDouble();
    double angle = hoodAngleMap.get(distanceMeters);
    desiredHoodAngle = angle;
    System.out.println("Setting hood angle to: " + angle + " degrees");
    _hood.runPosition(
        Angle.ofBaseUnits(angle * ShooterConstants.SIM_MULTIPLIER, Degrees),
        ShooterRotaryConstants.CRUISE_VELOCITY,
        ShooterRotaryConstants.ACCELERATION,
        ShooterRotaryConstants.JERK,
        PIDSlot.SLOT_0);
  }


  public void periodic() {
    _hood.periodic();
    _flywheel.periodic();
    _feeder.periodic();
    Logger.recordOutput("Flywheel/TargetVelocity", targetVelocity);
    Logger.recordOutput("Feeder/TargetVelocity", feederTargetVelocity);
    Logger.recordOutput("Hood/position", _hood.getPosition());
    Logger.recordOutput("Hood/desired_position", desiredHoodAngle);
    Logger.recordOutput("Hood/current", _hood.getSupplyCurrent());
    Logger.recordOutput("ShooterTuning/FlywheelRPM", _flywheel.getVelocity().in(RotationsPerSecond) * 60.0);
    Logger.recordOutput("ShooterTuning/HoodAngleDegrees", _hood.getPosition().in(Degrees));
    Logger.recordOutput("ShooterTuning/TargetRPM", targetVelocity.in(RotationsPerSecond) * 60.0);
    // _feeder.periodic();
    // _flywheel.periodic();

    // double pitch =
    //     Math.toRadians(
    //         Math.abs(Math.sin(Timer.getFPGATimestamp()) * 45)); // Placeholder for position

    // The pitch of the Rotation3D should be '_hood.getPosition().in(Radians)', change after fixing
    // motor configs.
    Logger.recordOutput(
        "3DField/3_Hood",
        new Pose3d(
            new Translation3d(-0.0075, 0.0, 0.523),
            new Rotation3d(0, _hood.getPosition().in(Radians), 0)));

    // _hood.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp()) * 0.25));
  }
}
