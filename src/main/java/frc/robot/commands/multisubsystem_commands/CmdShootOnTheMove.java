package frc.robot.commands.multisubsystem_commands;

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// -----------------------------------------------------
// Thank you to Team 3467 - Windham Windup!
// Code Adapted from 3467's velocityOffset.java
// https://github.com/WHS-FRC-3467/Skip-5.14-Nocturne/blob/PostGSDCleanup/src/main/java/frc/robot/Commands/velocityOffset.java
// -----------------------------------------------------

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.Hopper;
import frc.lib.firecontrol.ShotCalculator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class CmdShootOnTheMove extends Command {

  /** Creates a new CmdDriveShootOnTheMove. */
  private final Drive _drive;
  private final Shooter _shooter;

  private DoubleSupplier _translationXSupplier;
  private DoubleSupplier _translationYSupplier;
  private BooleanSupplier _trigger;
  private boolean _isFinished;

  private Translation2d _currentRobotTranslation;
  private double _currentAngleRadians;

  private Translation2d _futureRobotTranslation;
  private Rotation2d _futureAngleToSpeaker;

  private ChassisSpeeds _speeds;
  private double _correctedRotationRate;
  private double _timeUntilShot;
  private Translation2d _moveDelta;
  private Timer _shotTimer;
  private Boolean _hasRunOnce;
  private double _correctedRadius;
  private Pose2d _futureRobotPose2d;
  // private double _triggerThreshold;

  private PIDController _rotationPID;

  private ShotCalculator _shotCalc;
  private Hopper _hopper;

  /**
   * CmdAdjustShooterAutomatically is the default command for the shooter. Disable it when we run
   * this command.
   *
   * @param drivetrainSubsystem the drive subsystem
   * @param shooterSubsystem the shooter subsystem
   * @param hopperSubsystem the hopper subsystem
   * @param translationXSupplier translation x supplied by driver translation joystick
   * @param translationYSupplier translation y supplied by driver translation joystick []\
   * @param trigger the shoot button
   */
  public CmdShootOnTheMove(
      Drive drivetrainSubsystem,
      Shooter shooterSubsystem,
      Hopper hopperSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      BooleanSupplier trigger) {

    _drive = drivetrainSubsystem;
    _shooter = shooterSubsystem;
    _hopper = hopperSubsystem;
    _translationXSupplier = translationXSupplier;
    _translationYSupplier = translationYSupplier;
    _trigger = trigger;
    _shotCalc = _shooter.getShotCalculator();

    _rotationPID =
        new PIDController(
            Drive.SHOOT_ON_THE_MOVE_P, Drive.SHOOT_ON_THE_MOVE_I, Drive.SHOOT_ON_THE_MOVE_D);

    _rotationPID.setTolerance(Drive.SHOOT_ON_THE_MOVE_TOLERANCE);
    _rotationPID.enableContinuousInput(-Math.PI, Math.PI);

    _shotTimer = new Timer();
    _hasRunOnce = false;
    // _triggerThreshold = 0.1;

    addRequirements(drivetrainSubsystem, shooterSubsystem, hopperSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _isFinished = false;
    // Cancel CmdAdjustShooterAutomatically while this command runs; reenable when it finishes.
    _shooter.setAutoShootEnabled(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = _translationXSupplier.getAsDouble() * _drive.getMaxLinearSpeedMetersPerSec();
    double ySpeed = _translationYSupplier.getAsDouble() * _drive.getMaxLinearSpeedMetersPerSec();

    Translation2d hubCenter = _drive.getHubPos();
    Translation2d hubForward = (edu.wpi.first.wpilibj.DriverStation.getAlliance().orElse(edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) == edu.wpi.first.wpilibj.DriverStation.Alliance.Blue) 
            ? new Translation2d(1, 0) 
            : new Translation2d(-1, 0);

    ChassisSpeeds robotVel = _drive.getChassisSpeeds();
    ChassisSpeeds fieldVel = ChassisSpeeds.fromRobotRelativeSpeeds(robotVel, _drive.getPose().getRotation());

    ShotCalculator.ShotInputs inputs = new ShotCalculator.ShotInputs(
      _drive.getPose(),
      fieldVel,
      robotVel, // robot velocity
      hubCenter,
      hubForward, 
      1.0,  // Confidence
      0.0,  // pitch fallback
      0.0   // roll fallback
    );

    ShotCalculator.LaunchParameters shot = _shotCalc.calculate(inputs);

    org.littletonrobotics.junction.Logger.recordOutput("SOTM/Confidence", shot.confidence());
    org.littletonrobotics.junction.Logger.recordOutput("SOTM/IsValid", shot.isValid());
    org.littletonrobotics.junction.Logger.recordOutput("SOTM/Distance", shot.solvedDistanceM());
    org.littletonrobotics.junction.Logger.recordOutput("SOTM/RPM", shot.rpm());

    if (shot.isValid() && shot.confidence() > 50) {
      
      _shooter.setFlywheelVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(shot.rpm() / 60.0));
      _shooter.setHoodAngleImmediate(_shotCalc.getHoodAngle(shot.solvedDistanceM()));

      _rotationPID.setSetpoint(shot.driveAngle().getRadians());
    } else {
      _rotationPID.setSetpoint(_drive.getRotation2dToHub(_drive.getPose().getTranslation()).getRadians());
    }

    _currentAngleRadians = _drive.getRotation().getRadians();
    _correctedRotationRate = _rotationPID.calculate(MathUtil.inputModulus(_currentAngleRadians, -1 * Math.PI, Math.PI));

    _drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, _correctedRotationRate, _drive.getRotation()));

    if (_trigger.getAsBoolean() && shot.isValid() && shot.confidence() > 50) {
      // _shooter.runFeeder(edu.wpi.first.units.Units.RotationsPerSecond.of(30));
      // _hopper.runSpindexerImmediate(edu.wpi.first.units.Units.RotationsPerSecond.of(15));
    } else {
      _shooter.runFeeder(edu.wpi.first.units.Units.RotationsPerSecond.of(0));
      _hopper.runSpindexerImmediate(edu.wpi.first.units.Units.RotationsPerSecond.of(0));
    }

    // Keep state machine alive
    if (!_hasRunOnce) {
        _shotTimer.start();
        _hasRunOnce = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            _translationXSupplier.getAsDouble(),
            _translationYSupplier.getAsDouble(),
            0,
            _drive.getRotation()));
    _shotTimer.stop();
    _shotTimer.reset();
    _hasRunOnce = false;

    _shooter.runFeeder(edu.wpi.first.units.Units.RotationsPerSecond.of(0));
    _hopper.runSpindexerImmediate(edu.wpi.first.units.Units.RotationsPerSecond.of(0));
    _shooter.setFlywheelVelocity(edu.wpi.first.units.Units.RotationsPerSecond.of(0));
    // Reenable CmdAdjustShooterAutomatically because this command is finished.
    _shooter.setAutoShootEnabled(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _isFinished;
  }

  public double getHoodAngleDegrees(Translation2d robotPos) {

    Alliance alliance = Alliance.Blue; // default
    Translation2d hubPosition;

    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get();
    }

    if (alliance == Alliance.Red) {
      hubPosition = FieldConstants.OPPHUBCENTER;
    } else {
      hubPosition = FieldConstants.HUBCENTER;
    }

    double distance = robotPos.getDistance(hubPosition);

    double check =
        Math.pow(ShooterConstants.EXIT_VELOCITY, 4)
            - ShooterConstants.GRAVITY
                * (ShooterConstants.GRAVITY * Math.pow(distance, 2)
                    + 2
                        * ShooterConstants.HEIGHT_DIFFERENCE
                        * Math.pow(ShooterConstants.EXIT_VELOCITY, 2));

    if (check < 0) {
      return ShooterConstants.IDLE_HOOD_ANGLE; // Default angle if the shot is not possible
    }

    return Math.toDegrees(
        Math.atan(
            (ShooterConstants.EXIT_VELOCITY * ShooterConstants.EXIT_VELOCITY + Math.sqrt(check))
                / (ShooterConstants.GRAVITY * distance)));
  }

  // private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
  //   // Apply deadband
  //   double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
  //   Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

  //   // Square magnitude for more precise control
  //   linearMagnitude = linearMagnitude * linearMagnitude;

  //   // Return new linear velocity
  //   return new Pose2d(new Translation2d(), linearDirection)
  //       .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
  //       .getTranslation();
  // }
}
