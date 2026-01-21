package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;

public class Hopper extends SubsystemBase {

  private FlywheelMechanism _io;

  public Hopper(FlywheelMechanism io) {
    _io = io;
  }

  public void setGoal(double position) {
    Distance positionInches = Inches.of(position);
    _io.runPosition(HopperConstants.CONVERTER.toAngle(positionInches), PIDSlot.SLOT_0);
  }

  @Override
  public void periodic() {}
}
