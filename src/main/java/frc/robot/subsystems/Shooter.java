package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterFeederConstants;
import frc.robot.Constants.ShooterTowerConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final FlywheelMechanism _rflywheel;
  private final FlywheelMechanism _lflywheel;
  private final FlywheelMechanism _feeder;
  private final FlywheelMechanism _tower;
  private final RotaryMechanism _hood;

  // desired target values
  private double desiredVelo;
  private double hoodAngle;

  public Shooter(
      FlywheelMechanism lflywheel,
      FlywheelMechanism rflywheel,
      FlywheelMechanism feeder,
      FlywheelMechanism tower,
      RotaryMechanism hood) {
    _lflywheel = lflywheel;
    _rflywheel = rflywheel;
    _feeder = feeder;
    _tower = tower;
    _hood = hood;
  }

  // Sets feeder motor speed
  public void runFeeder() {
    _feeder.runVelocity(
        ShooterFeederConstants.MAX_VELOCITY,
        ShooterFeederConstants.MAX_ACCELERATION,
        PIDSlot.SLOT_2);
  }

  // Sets tower motor speed
  public void runTower() {
    _feeder.runVelocity(
        ShooterTowerConstants.MAX_VELOCITY, ShooterTowerConstants.MAX_ACCELERATION, PIDSlot.SLOT_2);
  }

  // Sets the flywheel velocity based on an input.
  public void setFlywheelVelocity(double velocity) {
    // store the desired velocity then send converted velocity to the mechanism
    this.desiredVelo = velocity;
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);
    AngularVelocity negangVelo = RotationsPerSecond.of(velocity);
    _lflywheel.runVelocity(angVelo, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
    _rflywheel.runVelocity(negangVelo, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  // // Broken aha !!
  // public enum State {
  //   OFF(Units.RevolutionsPerSecond.of(0.0)),
  //   IDLE(Units.RevolutionsPerSecond.of(ShooterConstants.IDLE_SPEED_RPM / 60)),
  //   SHOOT_FROM_HUB(Units.RevolutionsPerSecond.of(ShooterConstants.HUB_SPEED_RPM / 60)),
  //   SHOOT_FROM_TOWER(Units.RevolutionsPerSecond.of(ShooterConstants.TOWER_SPEED_RPM / 60)),
  //   SHOOT(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60)),
  //   SHOOT_ON_MOVE(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60));

  //   private final AngularVelocity stateVelocity;

  //   State(AngularVelocity stateVelocity) {
  //     this.stateVelocity = stateVelocity;
  //   }
  // }

  // Checks if the flywheel is at speed and returns a boolean
  public boolean flyAtVelocity() {
    return ((Math.abs(desiredVelo - _lflywheel.getVelocity().in(RotationsPerSecond))
                + Math.abs(desiredVelo - _rflywheel.getVelocity().in(RotationsPerSecond)))
            / 2)
        <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
  }

  public double getHoodAngleDegrees(Translation2d robotPos) {
    // Replace with HUB when it is added
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

  // Sets hood angle
  public void setHoodAngle(double angleDegrees) {
    hoodAngle = angleDegrees;
    _hood.runPosition(
        Angle.ofBaseUnits(angleDegrees, Degrees),
        ShooterConstants.HOOD_VELOCITY,
        ShooterConstants.HOOD_ACCELERATION,
        ShooterConstants.HOOD_JERK,
        PIDSlot.SLOT_0);
  }

  // Checks if hood is at angle
  public boolean hoodAtAngle() {
    return Math.abs(hoodAngle - _hood.getPosition().in(Degrees)) < ShooterConstants.HOOD_TOLERANCE;
  }

  public Command shoot(double velocity) {
    // Prepare targets
    return Commands.sequence(
        // Set and wait in parallel for both hood and flywheel
        Commands.parallel(
            Commands.run(() -> setFlywheelVelocity(velocity)).until(this::flyAtVelocity),
            Commands.run(() -> setHoodAngle(hoodAngle)).until(this::hoodAtAngle)),
        // feed once ready
        Commands.runOnce(() -> runFeeder()),
        Commands.runOnce(() -> runTower()),
        // stop flywheel when finished
        Commands.runOnce(() -> setFlywheelVelocity(0)));
  }

  public Command runFlywheel() {
    return Commands.runOnce(() -> setFlywheelVelocity(1));
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

    double flywheelSpeed =
        _lflywheel.getVelocity().magnitude() + _rflywheel.getVelocity().magnitude();

    double Yaw = Robot.robotContainer.drive.getPose().getRotation().getRadians();
    double V_xy =
        Math.sin(Math.PI / 2 - (_hood.getPosition().in(Radians) + Degrees.of(12).in(Radians)))
            * flywheelSpeed;

    // ChassisSpeeds driveChassisSpeeds = Robot.robotContainer.drive.getChassisSpeeds();
    // Translation3d driveSpeed3d = new Translation3d(
    //   0.0,
    //   0.0,
    //   0.0
    //   );

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
            Math.sin(Math.PI / 2 - (_hood.getPosition().in(Radians) + Degrees.of(12).in(Radians)))
                * flywheelSpeed));

    Robot.robotContainer.intake.simBalls--;
  }

  public void periodic() {
    _hood.periodic();
    _feeder.periodic();
    _tower.periodic();
    _lflywheel.periodic();
    _rflywheel.periodic();

    Logger.recordOutput(
        "3DField/3_Hood",
        new Pose3d(
            new Translation3d(-0.0075, 0.0, 0.523),
            new Rotation3d(0, _hood.getPosition().in(Radians), 0)));

    // For testing purposes, raises the hood
    // if (_hood.getPosition().in(Degrees) < ShooterRotaryConstants.MAX_ANGLE.in(Degrees) - 10)
    // _hood.runVoltage(Volts.of(7));

    // _hood.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp()) * 5));
    // _feeder.runVoltage(Volts.of(5));
    // _tower.runVoltage(Volts.of(5));
    // _flywheel.runVoltage(Volts.of(5));
  }
}
