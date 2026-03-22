package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.IntakePivotConstants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
  double velocity;
  double pivotAngle;
  public double desiredAngle;
  public int simBalls;
  public AngularVelocity targetSpeed = RotationsPerSecond.of(0);

  public Intake(FlywheelMechanism rollerIO, RotaryMechanism pivotIO) {
    _rollerIO = rollerIO;
    _pivotIO = pivotIO;

    simBalls = 0;
  }

  // Velocity of Rollers
  public void setVelocity(AngularVelocity velocity) {

    _rollerIO.runVelocity(velocity, IntakeFlywheelConstants.ACCELERATION, PIDSlot.SLOT_0);
    targetSpeed = velocity;
  }

  public Command setPivotAngle(Angle pivotAngle) {
    return this.runOnce(
        () ->
            _pivotIO.runPosition(
                pivotAngle,
                IntakePivotConstants.CRUISE_VELOCITY,
                IntakePivotConstants.ACCELERATION,
                IntakePivotConstants.JERK,
                PIDSlot.SLOT_0)
        );
  }

  public AngularVelocity getVelocity() {
    return _rollerIO.getVelocity();
  }

  public Angle getPosition() {
    return _pivotIO.getPosition();
  }

  public void stop() {
    setVelocity(RotationsPerSecond.of(0));
  }

  public Command runRollers(AngularVelocity velocity) {
    return Commands.run(() -> setVelocity(velocity));
  }

  public Command intake() {
    return Commands.parallel(
        runRollers(RotationsPerSecond.of(IntakeFlywheelConstants.PICKUP_SPEED)),
        setPivotAngle(IntakePivotConstants.PICKUP_ANGLE));
  }

  public boolean isIntendedAngle() {
    return Math.abs(desiredAngle - _pivotIO.getPosition().in(Degrees))
        <= IntakePivotConstants.TOLERANCE.magnitude();
  }

  public boolean canIntake() {
    return _pivotIO.getPosition().in(Degree) > (IntakePivotConstants.MAX_ANGLE.in(Degree) - 10)
        && simBalls < 45
        && simBalls >= 0;
  }

  public Command stowAndStopRollers() {
    return Commands.parallel(runRollers(RotationsPerSecond.of(0.0)), setStowAngle());
  }

  private Command setStowAngle() {
    return this.runOnce(
        () ->
            _pivotIO.runPosition(
                IntakePivotConstants.STOW_ANGLE,
                IntakePivotConstants.CRUISE_VELOCITY,
                IntakePivotConstants.ACCELERATION,
                IntakePivotConstants.JERK,
                PIDSlot.SLOT_0));
  }

  public Command jostleIntake() {
    return Commands.sequence(
        setPivotAngle(IntakePivotConstants.JOSTLE_ANGLE),
        new WaitCommand(0.5),
        setPivotAngle(IntakePivotConstants.PICKUP_ANGLE));
  }

  public void tunePivotPosition() {
    // DISABLED: Encoder was disabled in Constants to fix NT flooding in sim
    // System.out.println(IntakePivotConstants.ENCODER1.get());
    // _pivotIO.setEncoderPosition(Rotations.of(IntakePivotConstants.ENCODER1.get()));
  }

  public Command zeroEncoder() {
    return Commands.runOnce(() -> _pivotIO.setEncoderPosition(Degrees.of(0)));
  }

  @Override
  public void periodic() {
    // if (_pivotIO.getPosition().in(Degree) < IntakePivotConstants.MAX_ANGLE.in(Degree))
    //   _pivotIO.runVoltage(Volts.of(0.25));
    _rollerIO.periodic();
    Logger.recordOutput("Intake/TargetSpeed", targetSpeed);

    _pivotIO.periodic();
    Logger.recordOutput("Intake/TargetPivot", _pivotIO.getPosition().in(Degrees));
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

    // _pivotIO.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp())*0.25)); //--- Tests the pivot
  }
}
