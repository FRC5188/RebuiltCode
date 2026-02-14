package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

public class BallCounter extends SubsystemBase {
  private LaserCan _laserCAN;
  private Timer timer;
  private int ballsFired;
  private Timer secondsSinceLastFire;
  private List<Double> ballsData;
  private boolean isBlocked;
  private boolean canFire;
  private double fireRate;

  public BallCounter(LaserCan laserCan) {
    _laserCAN = laserCan;

    timer = new Timer();
    timer.start();

    ballsFired = 0;
    secondsSinceLastFire = new Timer();

    isBlocked = false;
    canFire = false;

    fireRate = 0.0;

    ballsData = new ArrayList<>();

    // Config for the LaserCAN
    // try {
    //     _laserCAN.setRangingMode(LaserCan.RangingMode.SHORT);
    //     // _laserCAN.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
    //     // _laserCAN.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    //     // } catch (ConfigurationFailedException e) {
    //     // System.out.println("Configuration failed! " + e);
    // }
  }

  public void shootBall() {
    ballsFired++;
    secondsSinceLastFire.reset();
    secondsSinceLastFire.start();

    ballsData.add(timer.get());

    System.out.println("Ball number " + (ballsFired) + " fired!");
  }

  public boolean ballShot() {
    Measurement measurement = _laserCAN.getMeasurement();
    return measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && measurement.distance_mm <= 30;
  }

  public double calculateFireRate() {
    double observationTime = 1.0;

    if (ballsData.size() <= 0) return fireRate;
    if (timer.get() - ballsData.get(0) > observationTime) ballsData.remove(0);

    fireRate = ballsData.size() / observationTime;
    return ballsData.size() / observationTime;
  }

  public int getBallsShot() {
    return ballsFired;
  }

  public double secondsSinceLastFired() {
    return secondsSinceLastFire.get();
  }

  @Override
  public void periodic() {
    isBlocked = ballShot();
    fireRate = calculateFireRate();

    secondsSinceLastFire.get();

    SmartDashboard.putNumber("Balls Fired", getBallsShot());
    SmartDashboard.putNumber("Ball Fire Rate (per second)", Math.round(fireRate * 100.0) / 100.0);
    SmartDashboard.putNumber("Seconds Since Last Fire", Math.round(secondsSinceLastFired() * 100.0) / 100.0);

    if (!isBlocked && !canFire) {
      canFire = true;
    }
    else if (isBlocked && canFire) {
      shootBall();
      canFire = false;
    }
  }
}
