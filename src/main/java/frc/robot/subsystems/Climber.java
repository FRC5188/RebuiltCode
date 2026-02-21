package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private LinearMechanism _io;

  public Climber(LinearMechanism io) {
    io = _io;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("3DField/4_Climber", new Pose3d(new Translation3d(0,0, ClimberConstants.CONVERTER.toDistance(_io.getPosition()).in(Meters)), new Rotation3d(0, 0, 0)));
  }
}
