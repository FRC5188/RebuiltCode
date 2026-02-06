package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private FlywheelMechanism _rollerIO;
  private RotaryMechanism _pivotIO;
  private LaserCan _laserCAN;

  public Intake(FlywheelMechanism rollerIO, RotaryMechanism pivotIO, LaserCan laserCan) {
    _rollerIO = rollerIO;
    _pivotIO = pivotIO;
    _laserCAN = laserCan;

    // Config for the LaserCAN
    /* try {
        lc.setRangingMode(LaserCan.RangingMode.SHORT);
        lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
    } */
  }

  // Velocity of Rollers
  public void setVelocity(double velocity) {
    AngularVelocity angVelo = RotationsPerSecond.of(velocity);

    _rollerIO.runVelocity(angVelo, Constants.IntakeConstants.ACCELERATION, PIDSlot.SLOT_0);
  }

  public AngularVelocity getVelocity() {
    return _rollerIO.getVelocity();
  }

  public Angle getPosition() {
    return _pivotIO.getPosition();
  }

  public void stop() {
    setVelocity(0);
  }

  public void printTargetDistance() {
    Measurement measurement = _laserCAN.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      System.out.println("The target is " + measurement.distance_mm + "mm away!");
    } else {
      System.out.println(
          "Oh no! The target is out of range, or we can't get a reliable measurement!");
    }
  }

  @Override
  public void periodic() {}
}
