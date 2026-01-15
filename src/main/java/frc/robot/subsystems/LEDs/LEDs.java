package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.w8.io.lights.LightsIOCandle;

public class LEDs extends SubsystemBase {
    private final LightsIOCandle _io;

    public LEDs(LightsIOCandle io) {
        _io = io;
    }

    @Override
    public void periodic() {}
}