package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.lights.LightsIO;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.hardware.CANdle;
import frc.robot.LEDConstants;

public class LEDs extends SubsystemBase {
    private final LightsIO _io;
    private final CANdle _candle;

    public LEDs(LightsIO io) {
        _io = io;
        _candle = new CANdle(1, "rio");
    }

    public Command runAnimation() {
        return this.startEnd(
            () -> _candle.setControl(LEDConstants.rainbowAnim.withSlot(0).withColor(LEDConstants.colorPaleBlue)),
            () -> _candle.setControl(new SolidColor(0, 3).withColor(colorWheezerBlue))
        )
    }

  @Override
  public void periodic() {}
}
