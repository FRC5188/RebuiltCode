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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.lib.W8.util.LoggerHelper;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
  double velocity;
  double pivotAngle;
  public double desiredAngle;
  public int simBalls;

  public Intake(FlywheelMechanism rollerIO, RotaryMechanism pivotIO) {
    _rollerIO = rollerIO;
    _pivotIO = pivotIO;

    simBalls = 0;
  }

  // Velocity of Rollers
  public void setVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _rollerIO.runVelocity(angVelo, IntakeFlywheelConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  public void setPivotAngle(Angle pivotAngle) {
    _pivotIO.runPosition(
        pivotAngle,
        IntakeFlywheelConstants.CRUISE_VELOCITY,
        IntakeFlywheelConstants.ACCELERATION,
        IntakeFlywheelConstants.JERK,
        PIDSlot.SLOT_0);
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
        Commands.runOnce(() -> this.setPivotAngle(IntakePivotConstants.PICKUP_ANGLE)),
        Commands.runOnce(() -> this.setVelocity(IntakeFlywheelConstants.PICKUP_SPEED)));
  }

  public boolean isIntendedAngle() {
    return Math.abs(desiredAngle - _pivotIO.getVelocity().in(RotationsPerSecond))
        <= IntakePivotConstants.TOLERANCE.magnitude();
  }

  public boolean canIntake() {
    return _pivotIO.getPosition().in(Degree) > (IntakePivotConstants.MAX_ANGLE.in(Degree) - 10)
        && simBalls < 45
        && simBalls >= 0;
  }

  public Command stowAndStopRollers() {
    return Commands.sequence(
        Commands.run(() -> setVelocity(IntakeFlywheelConstants.PICKUP_SPEED)),
        setStowAngle(IntakePivotConstants.STOW_ANGLE));
  }

  private Command setStowAngle(Angle stowAngle) {
    return this.runOnce(
        () ->
            _pivotIO.runPosition(
                stowAngle,
                IntakePivotConstants.CRUISE_VELOCITY,
                IntakePivotConstants.ACCELERATION,
                IntakePivotConstants.JERK,
                PIDSlot.SLOT_0));
  }

  @Override
  public void periodic() {
    LoggerHelper.recordCurrentCommand("1_Intake", this);

    _pivotIO.periodic();
    _rollerIO.periodic();

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

    // For testing purposes, opens hopper in sim
    // if (_pivotIO.getPosition().in(Degrees) < IntakePivotConstants.MAX_ANGLE.in(Degrees) - 10)
    // _pivotIO.runVoltage(Volts.of(7));

    // _pivotIO.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp())*5.0)); //--- Tests the pivot
    // _rollerIO.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp())*5.0)); //--- Tests the pivot
  }
}
