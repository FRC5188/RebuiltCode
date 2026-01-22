package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.robot.Constants.HopperConstants;

public class Hopper extends SubsystemBase {
    private FlywheelMechanism _io;

    public enum State
    {
        OFF (RevolutionsPerSecond.of(0.0)),
        FORWARD_SLOW (RevolutionsPerSecond.of(HopperConstants.SLOW_SPEED_RPM/60)),
        FORWARD_FAST (RevolutionsPerSecond.of(HopperConstants.FAST_SPEED_RPM/60)),
        REVERSE (RevolutionsPerSecond.of(HopperConstants.REVERSE_SPEED_RPM/60));

        private final AngularVelocity _stateVelocity;

        private State(AngularVelocity stateVelocity) {
            _stateVelocity = stateVelocity;
        }
    }

    public Hopper(FlywheelMechanism io) {
        _io = io;
    }

    @Override
    public void periodic() {
        
    }

}
