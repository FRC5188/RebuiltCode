package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;

public class Intake extends SubsystemBase {
    private FlywheelMechanism _io;

    public Intake(FlywheelMechanism io) {
        _io = io;
    }


    @Override
    public void periodic() {

    }
    
}
