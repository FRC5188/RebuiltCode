package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private LinearMechanism _io;

  public Climber(LinearMechanism io) {
    io = _io;
  }

  public void Position(double position) {

    Distance positionInches = Inches.of(position);
    _io.runPosition(
        ClimberConstants.CONVERTER.toAngle(positionInches),
        ClimberConstants.ANGULAR_VELOCITY,
        ClimberConstants.ANGULAR_ACCELERATION,
        null,
        PIDSlot.SLOT_0);
  }

  @Override
  public void periodic() {}
}
