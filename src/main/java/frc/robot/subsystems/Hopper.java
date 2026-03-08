package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.HopperConstants;

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

  // Velocity of Rollers
  public void setVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _io.runVelocity(angVelo, HopperConstants.ACCELERATION, PIDSlot.SLOT_0);
    targetVelocity = angVelo;
  }

  public Command runSpindexer(double velocity) {
    return Commands.runOnce(() -> setVelocity(velocity), this);
  }

  @Override
  public void periodic() {
    _io.periodic();
    Logger.recordOutput("Hopper/TargetVelocity", targetVelocity);
  }
}
