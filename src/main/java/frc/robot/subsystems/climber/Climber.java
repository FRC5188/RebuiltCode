package frc.robot.subsystems.climber;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private LinearMechanism _io;

  public Climber(LinearMechanism io) {
    io = _io;
  }

  public enum State {
    IDLE(Units.MetersPerSecond.of(0.0)),
    ASCENDING(Units.MetersPerSecond.of(ClimberConstants.CLIMBER_SPEED)),
    DESCENDING(Units.MetersPerSecond.of(-ClimberConstants.CLIMBER_SPEED));

    private final LinearVelocity stateVelocity;
    
    private State(LinearVelocity stateVelocity) {
      this.stateVelocity = stateVelocity;
    }
  }

  @Override
  public void periodic() {}
}
