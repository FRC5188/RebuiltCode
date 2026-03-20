package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private Debouncer homedDebounce = new Debouncer(0.1, DebounceType.kRising);
  private Trigger homedTrigger;
  private AngleUnit Degrees;
  private VoltageUnit Volts;
  private LinearMechanism _io;
  RotaryMechanism climber;
  Distance goalDistance;
  SwerveSetpoint STOW;
  SwerveSetpoint setpoint;

  public Climber(LinearMechanism io) {
    _io = io;
    homedTrigger =
        new Trigger(
            () ->
                homedDebounce.calculate(
                    _io.getSupplyCurrent().gte(Amps.of(ClimberConstants.HARD_STOP_CURRENT_LIMIT))));
  }

  // public void Position(double position) {

  //   Distance positionInches = Inches.of(position);
  //   _io.runPosition(
  //       ClimberConstants.CONVERTER.toAngle(positionInches),
  //       ClimberConstants.ANGULAR_VELOCITY,
  //       ClimberConstants.ANGULAR_ACCELERATION,
  //       null,
  //       PIDSlot.SLOT_0);
  // }

  public boolean isAboveCurrentLimit() {
    if (_io.getSupplyCurrent().in(Amps) > ClimberConstants.HARD_STOP_CURRENT_LIMIT) {
      return true;
    } else {
      return false;
    }
  }

  public Command runClimber() {
    return this.runOnce(
        () ->
            _io.runPosition(
                ClimberConstants.CONVERTER.toAngle(ClimberConstants.TOP),
                ClimberConstants.CRUISE_VELOCITY,
                ClimberConstants.ACCELERATION,
                ClimberConstants.JERK,
                PIDSlot.SLOT_0));
  }

  public Command calibrateClimber() {
    return Commands.sequence(
        runOnce(() -> _io.runVoltage(Voltage.ofBaseUnits(-1, Volts))),
        Commands.waitUntil(homedTrigger),
        runOnce(() -> _io.setEncoderPosition(Angle.ofBaseUnits(0, Degrees))),
        runOnce(() -> _io.runVoltage(Voltage.ofBaseUnits(0, Volts))));
  }

  public Command stopClimber() {
    return this.run(
        () ->
            _io.runVelocity(
                DegreesPerSecond.of(0.0), ClimberConstants.ACCELERATION, PIDSlot.SLOT_0));
  }

  public Command raiseClimber() {
    return this.run(
            () ->
                _io.runVelocity(
                    ClimberConstants.CRUISE_VELOCITY,
                    ClimberConstants.ACCELERATION,
                    PIDSlot.SLOT_0))
        .until(() -> isAboveCurrentLimit());
  }

  public Command lowerClimber() {
    System.out.println(ClimberConstants.LOWER_VELOCITY);
    System.out.println(ClimberConstants.ACCELERATION);
    return this.run(
            () ->
                _io.runVelocity(
                    ClimberConstants.LOWER_VELOCITY, ClimberConstants.ACCELERATION, PIDSlot.SLOT_0))
        .until(() -> isAboveCurrentLimit());
  }

  public boolean nearGoalposition() {
    if (Math.abs(
            goalDistance.in(Meters)
                - ClimberConstants.CONVERTER.toDistance(_io.getPosition()).in(Meters))
        < ClimberConstants.TOLERANCE.in(Meters)) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    _io.periodic();

    // double z = Math.abs(Math.sin(Timer.getFPGATimestamp()) * 0.33); // Placeholder for position

    // // The z of the Translation3D should be
    // // 'ClimberConstants.CONVERTER.toDistance(_io.getPosition()).in(Meters)', change after fixing
    // // motor configs.
    // Logger.recordOutput(
    //     "3DField/4_Climber", new Pose3d(new Translation3d(0, 0, z), new Rotation3d(0, 0, 0)));

    // _io.runVoltage(Volts.of(Math.sin(Timer.getFPGATimestamp()) * 0.25));
  }
}
