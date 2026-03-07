package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  private final FlywheelMechanism _rflywheel;
  private final FlywheelMechanism _lflywheel;
  private final FlywheelMechanism _feeder;
  private final RotaryMechanism _hood;

  // desired target values
  private double desiredVelo;
  private double hoodAngle;

  public Shooter(
      FlywheelMechanism lflywheel,
      FlywheelMechanism rflywheel,
      FlywheelMechanism feeder,
      RotaryMechanism hood) {
    _lflywheel = lflywheel;
    _rflywheel = rflywheel;
    _feeder = feeder;
    _hood = hood;
  }

  // Sets feeder motor speed
  public void runFeeder(AngularVelocity velocity) {
    _feeder.runVelocity(
        velocity, FeederConstants.FEED_ACCELERATION, PIDSlot.SLOT_1);
  }

  // Sets the flywheel velocity based on an input.
  public void setFlywheelVelocity(AngularVelocity velocity) {
    // store the desired velocity then send converted velocity to the mechanism
    // this.desiredVelo = velocity;
    // AngularVelocity angVelo = RotationsPerSecond.of(velocity);
    // AngularVelocity negangVelo = RotationsPerSecond.of(velocity);
    _lflywheel.runVelocity(velocity, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
    _rflywheel.runVelocity(velocity, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
    Logger.recordOutput("LeftFlywheel/TargetSpeed",velocity);
    Logger.recordOutput("RightFlywheel/TargetSpeed",velocity);

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
    return ((Math.abs(desiredVelo - _lflywheel.getVelocity().in(RotationsPerSecond))
                + Math.abs(desiredVelo - _rflywheel.getVelocity().in(RotationsPerSecond)))
            / 2)
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

  public Command shoot(AngularVelocity velocity) {
    // Prepare targets
    return Commands.sequence(
        // Set and wait in parallel for both hood and flywheel
        Commands.parallel(
            Commands.run(() -> setFlywheelVelocity(velocity)).until(this::flyAtVelocity),
            Commands.run(() -> setHoodAngle(hoodAngle)).until(this::hoodAtAngle)),
        // feed once ready
        Commands.runOnce(() -> runFeeder(FeederConstants.FEED_SPEED)),
        // stop flywheel when finished
        Commands.runOnce(() -> setFlywheelVelocity(RotationsPerSecond.of(0.0))));
  }

  public Command runFlywheel(AngularVelocity velocity) {
        System.out.println("Flywheel");

    return Commands.runOnce(() -> setFlywheelVelocity(velocity));
  }

  public Command runTower(AngularVelocity velocity) {
    System.out.println("Tower");
    return Commands.runOnce(() -> runFeeder(velocity));
  }

  public void simShoot() {
    if (Robot.robotContainer.intake.simBalls <= 0) return;

    double flywheelSpeed = 6;
    Translation2d robotPose2d = Robot.robotContainer.drive.getPose().getTranslation();
    double Yaw = Robot.robotContainer.drive.getPose().getRotation().getRadians();
    Pose3d robotPose3d =
        new Pose3d(
            new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0),
            new Rotation3d(robotPose2d.getAngle()));
    Pose3d shooterPose3d =
        new Pose3d(
            new Translation3d(-0.0075, 0.0, 0.523),
            new Rotation3d(0, _hood.getPosition().in(Radians), 0));

    double V_xy =
        Math.sin(Math.PI / 2 - (_hood.getPosition().in(Radians) + Degrees.of(12).in(Radians)))
            * flywheelSpeed;

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
    _lflywheel.periodic();
    _rflywheel.periodic();
    _feeder.periodic();
    // _feeder.periodic();
    // _flywheel.periodic();

    // double pitch =
    //     Math.toRadians(
    //         Math.abs(Math.sin(Timer.getFPGATimestamp()) * 45)); // Placeholder for position

    // The pitch of the Rotation3D should be '_hood.getPosition().in(Radians)', change after fixing
    // motor configs.
    // Logger.recordOutput(
    //     "3DField/3_Hood",
    //     new Pose3d(new Translation3d(-0.0075, 0.0, 0.523), new Rotation3d(0, pitch, 0)));

    // _hood.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp()) * 0.25));
  }
}
