package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private LinearMechanism _io;
  Distance goalDistance;

  public Climber(LinearMechanism io) {
    io = _io;
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

  @Override
  public void periodic() {}

  public void runClimber() {
    runClimber();
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
}
