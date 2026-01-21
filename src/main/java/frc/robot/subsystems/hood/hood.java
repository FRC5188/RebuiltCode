package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism;

public class hood extends SubsystemBase {
  private final LinearMechanism _io;

  public hood(LinearMechanism io) {
    _io = io;
  }

  @Override
  public void periodic() {}
}
