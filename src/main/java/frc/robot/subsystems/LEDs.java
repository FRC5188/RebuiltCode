package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.lights.LightsIO;

public class LEDs extends SubsystemBase {
  private final LightsIO _io;

  public LEDs(LightsIO io) {
    _io = io;
  }

  @Override
  public void periodic() {}
}
