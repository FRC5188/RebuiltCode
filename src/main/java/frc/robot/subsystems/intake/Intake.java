package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.Slot1Configs;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private FlywheelMechanism _io;
    

    public Intake(FlywheelMechanism io) {
        _io = io;
    }

    public void setVelocity(double velocity) {
        AngularVelocity angVelo = RotationsPerSecond.of(velocity);

        _io.runVelocity(angVelo, Constants.ACCELERATION, PIDSlot.SLOT_0);
    }

    @Override
    public void periodic() {

    }
    
}
