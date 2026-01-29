package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  private FlywheelMechanism _flywheel;
  private FlywheelMechanism _feeder;
  public double desiredVelo;
  public Angle HoodAngle;

  public Shooter(FlywheelMechanism flywheel, FlywheelMechanism feeder) {
    _flywheel = flywheel;
    _feeder = feeder;
  }

  // Sets feeder motor speed
  public void runFeeder() {
    _feeder.runVelocity(
        FeederConstants.FEED_SPEED, FeederConstants.FEED_ACCELERATION, PIDSlot.SLOT_2);
  }

  // Sets the flywheel velocity based on an input.
  public void setFlywheelVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);
    velocity = desiredVelo;

    _flywheel.runVelocity(angVelo, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  // Checks if the flywheel is at speed and returns a boolean
  public boolean flyAtVelocity() {
    return Math.abs(desiredVelo - _flywheel.getVelocity().in(RotationsPerSecond))
        <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
  }

  public double getHoodAngleDegrees(double distanceToTarget) {

    final double g = 9.81;

    double check = Math.pow(ShooterConstants.EXIT_VELOCITY, 4) - g * (g * Math.pow(distanceToTarget, 2) + 2 * ShooterConstants.HEIGHT_DIFFERENCE * Math.pow(ShooterConstants.EXIT_VELOCITY, 2));

    if (check < 0) {
      return 45.0; // Default angle if the shot is not possible
    }
    return Math.toDegrees(Math.atan((ShooterConstants.EXIT_VELOCITY*ShooterConstants.EXIT_VELOCITY + Math.sqrt(check)) / (g * distanceToTarget)));
  }

  public Command shoot(double velocity) {
    return Commands.run(
            () -> {
              setFlywheelVelocity(velocity);
            })
        .until(
            () -> flyAtVelocity())
        .andThen(
            () -> {
              runFeeder();
            })
        .andThen(
          ()->{
            setFlywheelVelocity(0);
          });
  }
 
  @Override
  public void periodic() {}
}
