package frc.robot.subsystems.drive.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;

public class Shooter extends SubsystemBase {
    private FlywheelMechanism _io;

    public Shooter(FlywheelMechanism io) {
        _io = io;
    }

    @Override
    public void periodic() {

    }
}
