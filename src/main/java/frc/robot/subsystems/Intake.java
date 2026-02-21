package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Intake extends SubsystemBase {
  private FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
  public double desiredAngle;

  public Intake(FlywheelMechanism rollerIO, RotaryMechanism pivotIO) {
    _rollerIO = rollerIO;
    _pivotIO = pivotIO;
  }

  // Velocity of Rollers
  public void setVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _rollerIO.runVelocity(angVelo, Constants.IntakeConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  public Command setPivotAngle(Angle pivotAngle) {
    return this.runOnce(
        () ->
            _pivotIO.runPosition(
                pivotAngle,
                IntakeConstants.CRUISE_VELOCITY,
                IntakeConstants.ACCELERATION,
                IntakeConstants.JERK,
                PIDSlot.SLOT_0));
    // .withName("Go To " + setpoint.toString() + " Setpoint");
  }

  public AngularVelocity getVelocity() {
    return _rollerIO.getVelocity();
  }

  public Angle getPosition() {
    return _pivotIO.getPosition();
  }

  public void stop() {
    setVelocity(0);
  }

  public Command intake() {
    return Commands.sequence(
      Commands.run(() -> setVelocity(Constants.IntakeConstants.PICKUP_SPEED)), setPivotAngle(Constants.IntakeConstants.PICKUP_ANGLE));
   }

  public boolean isIntendedAngle() {
    return Math.abs(desiredAngle - _pivotIO.getVelocity().in(RotationsPerSecond))
        <= IntakePivotConstants.TOLERANCE.magnitude();
  }
 public Command stowAngle() { 
  return Commands.sequence(
    Commands.run(() -> setVelocity(Constants.IntakeConstants.PICKUP_SPEED)), setStowAngle(Constants.IntakeConstants.STOW_ANGLE)); 
     }
      private Command setStowAngle(Angle stowAngle) {
      return this.runOnce(
        () ->
            _pivotIO.runPosition(
                stowAngle,
                IntakeConstants.CRUISE_VELOCITY,
                IntakeConstants.ACCELERATION,
                IntakeConstants.JERK,
                PIDSlot.SLOT_0));
    }
    
      @Override
  public void periodic() {}
}
