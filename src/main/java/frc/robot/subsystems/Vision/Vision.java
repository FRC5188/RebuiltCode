package frc.robot.subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.lib.W8.io.vision.VisionIO;

public class Vision extends SubsystemBase {
    private final VisionIO _io;

    public Vision(VisionIO io) {
        _io = io;
    }

    @Override
    public void periodic() {

    }
}
