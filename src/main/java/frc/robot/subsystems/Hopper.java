package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

public class Hopper extends SubsystemBase {
  private FlywheelMechanism _io;

  public AngularVelocity targetVelocity = RotationsPerSecond.of(0.0);

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

  public Command runSpindexer(AngularVelocity velocity) {
    return Commands.runOnce(
        () -> {
          _io.runVelocity(velocity, HopperConstants.ACCELERATION, PIDSlot.SLOT_0);
          targetVelocity = velocity;
        },
        this);
  }

  @Override
  public void periodic() {
    _io.periodic();
    Logger.recordOutput("Hopper/TargetVelocity", targetVelocity);
  }
}
