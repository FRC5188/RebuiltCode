package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants.ShooterConstants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;

public class Shooter extends SubsystemBase {
  private FlywheelMechanism _io;

  public Shooter(FlywheelMechanism io) {
    _io = io;
  }

  public enum State {
    OFF(Units.RevolutionsPerSecond.of(0.0)), 
    IDLE(Units.RevolutionsPerSecond.of(ShooterConstants.IDLE_SPEED_RPM/60)), 
    SHOOT_FROM_HUB(Units.RevolutionsPerSecond.of(ShooterConstants.HUB_SPEED_RPM/60)), 
    SHOOT_FROM_TOWER(Units.RevolutionsPerSecond.of(ShooterConstants.TOWER_SPEED_RPM/60)), 
    SHOOT(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM/60)), 
    SHOOT_ON_MOVE(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM/60));
    
    private final AngularVelocity stateVelocity;
    State(AngularVelocity stateVelocity) {
      this.stateVelocity = stateVelocity;
    }
  }

  @Override
  public void periodic() {}
}
