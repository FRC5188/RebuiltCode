package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {

  private FlywheelMechanism _io;
  // public enum State {
  //   OFF(RevolutionsPerSecond.of(0.0)),
  //   FORWARD_SLOW(RevolutionsPerSecond.of(HopperConstants.SLOW_SPEED_RPM / 60)),
  //   FORWARD_FAST(RevolutionsPerSecond.of(HopperConstants.FAST_SPEED_RPM / 60)),
  //   REVERSE(RevolutionsPerSecond.of(HopperConstants.REVERSE_SPEED_RPM / 60));

  //   private final AngularVelocity _stateVelocity;

  //   private State(AngularVelocity stateVelocity) {
  //     _stateVelocity = stateVelocity;
  //   }
  // }

  public Hopper(FlywheelMechanism io) {
    _io = io;
  }

  public void setGoal(double position) {
    Distance positionInches = Inches.of(position);
    _io.runPosition(
        HopperConstants.CONVERTER.toAngle(positionInches),
        HopperConstants.ANGULAR_VELOCITY,
        HopperConstants.ANGULAR_ACCELERATION,
        null,
        PIDSlot.SLOT_0);
  }

  @Override
  public void periodic() {}
}
