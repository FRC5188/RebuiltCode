package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism;

public class Climber extends SubsystemBase {
  private LinearMechanism _io;

  public Climber(LinearMechanism io) {
    io = _io;
  }

  @Override
  public void periodic() {}
}
