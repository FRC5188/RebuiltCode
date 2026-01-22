package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.HopperCOnstants;

public class Hopper extends SubsystemBase {

    private RotaryMechanism _io;

    public Hopper(RotaryMechanism io) {
        _io = io;
    }

    public void runHopper() {
        _io.runVoltage(HopperConstants.VOLTAGE);
    }

    @Override
    public void periodic() {
        
    }

}
