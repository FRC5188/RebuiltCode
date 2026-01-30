package frc.robot.subsystems.hopper;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants;
import frc.robot.Constants.HopperConstants;

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
