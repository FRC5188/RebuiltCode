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

  public Command setPivotAngle(double pivotAngle) {
    return Commands.run(() -> setPivotAngle(pivotAngle));
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

  public void setAngle(Angle angle) {
    _pivotIO.runPosition(
        angle,
        getVelocity(),
        IntakePivotConstants.ACCELERATION,
        IntakePivotConstants.JERK,
        PIDSlot.SLOT_0);
    desiredAngle = angle.magnitude();
  }

  public boolean isIntendedAngle() {
    return Math.abs(desiredAngle - _pivotIO.getVelocity().in(RotationsPerSecond))
        <= IntakePivotConstants.TOLERANCE.magnitude();
  }

  @Override
  public void periodic() {}
}
