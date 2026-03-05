package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.IntakePivotConstants;

public class Hopper extends SubsystemBase {
  private FlywheelMechanism _io;

  public Hopper(FlywheelMechanism io) {
    _io = io;
  }

  public void runVelocity() {
    _io.runVelocity(HopperConstants.ANGULAR_VELOCITY, HopperConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  public Command runHopper() {
    return Commands.run(() -> runVelocity());
  }

  @Override
  public void periodic() {}
}
