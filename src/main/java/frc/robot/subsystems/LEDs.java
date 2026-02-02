package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.devices.Lights;
import frc.lib.W8.io.lights.LightsIO;
import edu.wpi.first.wpilibj2.command.Command;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import frc.robot.Constants.LEDConstants;

public class LEDs extends SubsystemBase {
    private final LightsIO _io;
    private final Lights _lights;

    public LEDs(LightsIO io, Lights lights) {
        _io = io;
        _lights = lights;
    }

    public Command runAnimation() {
        return this.startEnd(
            () -> _lights.setAnimation(new SolidColor(0, 3).withColor(LEDConstants.colorPaleBlue)),
            () -> _lights.setAnimation(new SolidColor(0, 3).withColor(LEDConstants.colorWheezerBlue))
        );
    }

  @Override
  public void periodic() {}
}