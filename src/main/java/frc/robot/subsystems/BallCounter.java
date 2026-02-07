package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;

public class BallCounter extends SubsystemBase {
  private LaserCan _laserCAN;
  private Timer timer;
  private int ballsFired;
  private Timer secondsSinceLastFire;
  private List<Double> ballsData;

  public BallCounter(LaserCan laserCan) {
    _laserCAN = laserCan;

    timer = new Timer();
    timer.start();

    ballsFired = 0;
    secondsSinceLastFire = new Timer();

    ballsData = new ArrayList<>();

    // Config for the LaserCAN
    /* try {
        lc.setRangingMode(LaserCan.RangingMode.SHORT);
        lc.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
        lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
        System.out.println("Configuration failed! " + e);
    } */
  }

  public void shootBall() {
    ballsFired++;
    secondsSinceLastFire.reset();

    ballsData.add(timer.get());

    System.out.println("Ball number " + (ballsFired) + " fired!");
  }

  public boolean ballShot() {
    Measurement measurement = _laserCAN.getMeasurement();
    return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT;
  }

  public double calculateFireRate() {
    double observationTime = 3.0;

    int iteration = 0;
    for (double ballTime : ballsData) {
      double timeInterval = timer.get() - ballTime;
      if (timeInterval <= observationTime) {
        iteration++;
      } else {
        ballsData.remove(iteration);
      }
    }

    return ballsData.size() / observationTime;
  }

  // Getter Methods
  public int getBallsShot() {
    return ballsFired;
  }

  public double secondsSinceLastFired() {
    return secondsSinceLastFire.get();
  }

  @Override
  public void periodic() {
    secondsSinceLastFire.get();

    SmartDashboard.putNumber("Balls Fired", getBallsShot());
    SmartDashboard.putNumber("Ball Fire Rate (per second)", calculateFireRate());
    SmartDashboard.putNumber("Seconds Since Last Fire", secondsSinceLastFired());

    if (ballShot()) {
      shootBall();
    }
    // else System.out.println("No target found!");
  }
}
// (ballsData.isEmpty() ? 0 : ballsData.size()+1) {ignore}
