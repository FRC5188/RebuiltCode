package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.lights.LightsIO;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.led.Animation;

public class LEDs extends SubsystemBase {
    private final LightsIO _io;

    public LEDs(LightsIO io) {
        _io = io;
    }

    // Lights is PROBABLY going to be a candle I THINK ?? (in the future)
    public Command runAnimation(Animation animation1, Animation animation2) {
        return this.startEnd(
            () -> lights.;
        )
    }

    @Override
    public void periodic() {}
}