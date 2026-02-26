package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
  double velocity;
  double pivotAngle;
  public int simBalls;
  public double desiredAngle;

  public Intake(FlywheelMechanism rollerIO, RotaryMechanism pivotIO) {
    _rollerIO = rollerIO;
    _pivotIO = pivotIO;

    simBalls = 0;
  }

  // Velocity of Rollers
  public void setVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _rollerIO.runVelocity(angVelo, Constants.IntakeFlywheelConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  public Command setPivotAngle(Angle pivotAngle) {
    return this.runOnce(
        () ->
            _pivotIO.runPosition(
                pivotAngle,
                IntakeFlywheelConstants.CRUISE_VELOCITY,
                IntakeFlywheelConstants.ACCELERATION,
                IntakeFlywheelConstants.JERK,
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
  //       Commands.run(() -> setVelocity(velocity)), Commands.run(() ->
  // setPivotAngle(pivotAngle)));
  // }
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

  public boolean canIntake()
  {
    return _pivotIO.getPosition().in(Degree) > (IntakePivotConstants.MAX_ANGLE.in(Degree) - 10) && simBalls < 45 && simBalls >= 0;
  }

  public void periodic() {
    if (_pivotIO.getPosition().in(Degree) < IntakePivotConstants.MAX_ANGLE.in(Degree)) _pivotIO.runVoltage(Volts.of(0.25));

    _pivotIO.periodic();
    Logger.recordOutput(
        "3DField/1_Intake",
        new Pose3d(
            new Translation3d(0.3085, 0.0, 0.175),
            new Rotation3d(0, _pivotIO.getPosition().in(Radians), 0)));
    Logger.recordOutput(
        "3DField/2_Hopper",
        new Pose3d(
            new Translation3d(Math.sin(_pivotIO.getPosition().in(Radians) * 0.1055), 0, 0),
            new Rotation3d(0, 0, 0)));

     //_pivotIO.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp())*0.25)); //--- Tests the pivot
  }
}
