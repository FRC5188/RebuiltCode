package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private final FlywheelMechanism _flywheel;
  private final FlywheelMechanism _feeder;
  private final RotaryMechanism _hood;

  // desired target values
  private double desiredVelo;
  private double hoodAngle;

  public Shooter(FlywheelMechanism flywheel, FlywheelMechanism feeder, RotaryMechanism hood) {
    _flywheel = flywheel;
    _feeder = feeder;
    _hood = hood;
  }

  // Sets feeder motor speed
  public void runFeeder() {
    _feeder.runVelocity(
        FeederConstants.FEED_SPEED, FeederConstants.FEED_ACCELERATION, PIDSlot.SLOT_2);
  }

  // Sets the flywheel velocity based on an input.
  public void setFlywheelVelocity(double velocity) {
    // store the desired velocity then send converted velocity to the mechanism
    this.desiredVelo = velocity;
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _flywheel.runVelocity(angVelo, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
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
    return Math.abs(desiredVelo - _flywheel.getVelocity().in(RotationsPerSecond))
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

  public Command shoot(double velocity) {
    // Prepare targets
    return Commands.sequence(
        // Set and wait in parallel for both hood and flywheel
        Commands.parallel(
            Commands.run(() -> setFlywheelVelocity(velocity)).until(this::flyAtVelocity),
            Commands.run(() -> setHoodAngle(hoodAngle)).until(this::hoodAtAngle)),
        // feed once ready
        Commands.runOnce(() -> runFeeder()),
        // stop flywheel when finished
        Commands.runOnce(() -> setFlywheelVelocity(0)));
  }

  @Override
  public void periodic() {}
}
