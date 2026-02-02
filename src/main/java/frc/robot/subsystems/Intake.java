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
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Intake.java
import frc.robot.Constants.IntakeConstants;
=======
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.ShooterConstants;
import edu.wpi.first.math.MathUtil;
>>>>>>> fd283d6 (Implemented Pivot Methods 2/2/26):src/main/java/frc/robot/subsystems/intake/Intake.java

public class Intake extends SubsystemBase {
  private FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
<<<<<<< HEAD:src/main/java/frc/robot/subsystems/Intake.java
  double velocity;
  double pivotAngle;
=======
  public double desiredAngle;
>>>>>>> fd283d6 (Implemented Pivot Methods 2/2/26):src/main/java/frc/robot/subsystems/intake/Intake.java

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

  // public Command intake() {
  //   return Commands.sequence(
  //       Commands.run(() -> setVelocity(velocity)), Commands.run(() -> setPivotAngle(pivotAngle)));
  // }
  public void setAngle(Angle angle)
  {
    _pivotIO.runPosition(angle, getVelocity(), IntakePivotConstants.ACCELERATION, IntakePivotConstants.JERK, PIDSlot.SLOT_0);
    desiredAngle = angle.magnitude();
  }

  public boolean isIntendedAngle()
  {
    return Math.abs(desiredAngle - _pivotIO.getVelocity().in(RotationsPerSecond))
        <= IntakePivotConstants.TOLERANCE.magnitude();
  }

  @Override
  public void periodic() {}
}