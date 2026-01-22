package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units;

public class Hopper extends SubsystemBase {
    private FlywheelMechanism _io;

    public enum State
    {
        OFF /*(Units.RevolutionsPerSecond.of(0.0))*/,
        FORWARD_SLOW /*(Units.RevolutionsPerSecond.of(SLOW_SPEED_RPM/60))*/,
        FORWARD_FAST /*(Units.RevolutionsPerSecond.of(FAST_SPEED_RPM/60))*/,
        REVERSE /*(Units.RevolutionsPerSecond.of(REVERSE_SPEED_RPM/60))*/;

        private final AngularVelocity _stateVelocity;

        public State(AngularVelocity stateVelocity) {
            _stateVelocity = stateVelocity;
        }

        // CREATE CONSTANTS FOR RPM OF EACH STATE IN HopperConstants !

    }

    public Hopper(FlywheelMechanism io) {
        _io = io;
    }

    @Override
    public void periodic() {
        
    }

}
