// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.W8.io.motor.*;
import frc.lib.W8.io.motor.MotorIO.PIDSlot;
import frc.lib.W8.io.vision.VisionIOPhotonVision;
import frc.lib.W8.io.vision.VisionIOPhotonVisionSim;
import frc.lib.W8.mechanisms.flywheel.*;
import frc.lib.W8.mechanisms.linear.LinearMechanism;
import frc.lib.W8.mechanisms.linear.LinearMechanismReal;
import frc.lib.W8.mechanisms.linear.LinearMechanismSim;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanismReal;
import frc.lib.W8.mechanisms.rotary.RotaryMechanismSim;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterRotaryConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final Drive drive;
  private final Hopper hopper;
  private final Shooter shooter;
  public final Intake intake;
  //   private final BallCounter ballCounter;
  private final Vision vision;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if the robot is real before using the ball counter!
    // if (Robot.isReal()) ballCounter = new BallCounter(new LaserCan(1));
    // else ballCounter = null;

    BaseStatusSignal.setUpdateFrequencyForAll(50.0);

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        hopper =
            new Hopper(
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        HopperConstants.MOTOR_NAME,
                        HopperConstants.getFXConfig(),
                        Ports.Spindexer)));
        shooter =
            new Shooter(
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        "ShooterRightFlywheel",
                        ShooterFlywheelConstants.getFXConfig(false),
                        Ports.RightFlywheel, ShooterFlywheelConstants.FOLLOWER_1)),
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        "ShooterTower", FeederConstants.getFXConfig(false), Ports.TowerRoller)),
                new RotaryMechanismReal(
                    new MotorIOTalonFX(
                        ShooterRotaryConstants.NAME,
                        ShooterRotaryConstants.getFXConfig(),
                        Ports.HoodMotor),
                    Constants.ShooterRotaryConstants.CONSTANTS,
                    java.util.Optional.empty()));

        intake =
            new Intake(
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        IntakeFlywheelConstants.MOTOR_NAME,
                        IntakeFlywheelConstants.getFXConfig(),
                        Ports.IntakeRoller)),
                new RotaryMechanismReal(
                    new MotorIOTalonFX(
                        IntakePivotConstants.NAME,
                        IntakePivotConstants.getFXConfig(),
                        Ports.IntakePivot),
                    IntakePivotConstants.CONSTANTS,
                    Optional.empty()));
        climber =
            new Climber(
                new LinearMechanismReal(
                    new MotorIOTalonFX(
                        ClimberConstants.MOTOR_NAME,
                        ClimberConstants.getFXConfig(),
                        Ports.ClimberMotor),
                    ClimberConstants.CHARACTERISTICS));
        vision =
            new Vision(
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.aprilTagLayout,
                    PoseStrategy.CONSTRAINED_SOLVEPNP));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        hopper =
            new Hopper(
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        HopperConstants.MOTOR_NAME, HopperConstants.getFXConfig(), Ports.Spindexer),
                    HopperConstants.DCMOTOR,
                    HopperConstants.MOI,
                    HopperConstants.TOLERANCE));
        shooter =
            new Shooter(
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(true),
                        Ports.RightFlywheel, ShooterFlywheelConstants.FOLLOWER_1),
                    ShooterFlywheelConstants.DCMOTOR,
                    ShooterFlywheelConstants.MOI,
                    ShooterFlywheelConstants.TOLERANCE),
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(false),
                        Ports.TowerRoller),
                    ShooterFlywheelConstants.DCMOTOR,
                    ShooterFlywheelConstants.MOI,
                    ShooterFlywheelConstants.TOLERANCE),
                new RotaryMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterRotaryConstants.NAME,
                        ShooterRotaryConstants.getFXConfig(),
                        Ports.HoodMotor),
                    ShooterRotaryConstants.DCMOTOR,
                    ShooterRotaryConstants.MOI,
                    true,
                    ShooterRotaryConstants.CONSTANTS,
                    java.util.Optional.empty()));

        intake =
            new Intake(
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        IntakeFlywheelConstants.MOTOR_NAME,
                        IntakeFlywheelConstants.getFXConfig(),
                        Ports.IntakeRoller),
                    IntakeFlywheelConstants.DCMOTOR,
                    IntakeFlywheelConstants.MOI,
                    IntakeFlywheelConstants.TOLERANCE),
                new RotaryMechanismSim(
                    new MotorIOTalonFXSim(
                        IntakePivotConstants.NAME,
                        IntakePivotConstants.getFXConfig(),
                        Ports.IntakeRoller),
                    IntakePivotConstants.DCMOTOR,
                    IntakePivotConstants.MOI,
                    false,
                    IntakePivotConstants.CONSTANTS,
                    Optional.empty()));
        vision =
            new Vision(
                new VisionIOPhotonVisionSim(
                    () -> drive.getPose(),
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.aprilTagLayout,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    VisionConstants.getSystemSim()));
        climber =
            new Climber(
                new LinearMechanismSim(
                    new MotorIOTalonFXSim(
                        ClimberConstants.MOTOR_NAME,
                        ClimberConstants.getFXConfig(),
                        Ports.ClimberMotor),
                    ClimberConstants.DCMOTOR,
                    ClimberConstants.CARRIAGE_MASS,
                    ClimberConstants.CHARACTERISTICS,
                    false));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        hopper = new Hopper(new FlywheelMechanism() {});

        shooter =
            new Shooter(
                new FlywheelMechanism() {},
                new FlywheelMechanism() {},
                new RotaryMechanism(null, null) {});

        intake =
            new Intake(
                new FlywheelMechanism() {},
                new RotaryMechanism(IntakePivotConstants.NAME, IntakePivotConstants.CONSTANTS) {});
        vision = new Vision(null);

        climber =
            new Climber(
                new LinearMechanism(ClimberConstants.NAME, ClimberConstants.CHARACTERISTICS) {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    // controller
    //     .b()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
    //                 drive)
    //             .ignoringDisable(true));

    // controller
    //     .x()
    //     .onTrue(Commands.runOnce(() -> hopper.setGoal(HopperConstants.HOPPER_POSITION), hopper));

    // controller.rightTrigger().onTrue(Commands.runOnce(() -> shooter.simShoot()));

    // controller
    //  .leftTrigger()
    // .onTrue(
    //  Commands.runOnce(
    //    () -> {
    //    Robot.fuelSim.clearFuel();
    //  Robot.fuelSim.spawnStartingFuel();
    //       intake.simBalls = 0;
    //   }));
    // controller.y().onTrue(Commands.runOnce(() -> intake.setVelocity(RotationsPerSecond.of(10))));
    // controller.a().onTrue(intake.intake());
    // controller.x().onTrue(intake.stowAndStopRollers());

    // Flywheel
    controller.leftBumper().whileTrue(shooter.runFlywheel(RotationsPerSecond.of(20)));
    controller.leftBumper().onFalse(shooter.runFlywheel(RotationsPerSecond.of(0)));

    // Intake + Spindexer + Tower
    controller
        .rightBumper()
        .whileTrue(
            Commands.parallel(
                // intake.runRollers(RotationsPerSecond.of(30)),
                hopper.runSpindexer(15), shooter.runTower(RotationsPerSecond.of(70))));

    controller
        .rightBumper()
        .onFalse(
            Commands.parallel(
                intake.runRollers(RotationsPerSecond.of(0)),
                hopper.runSpindexer(0),
                shooter.runTower(RotationsPerSecond.of(0))
                ));

    // Intake Rollers 11 Motor: 9 Intake
    
    controller.a().whileTrue((intake.runRollers(RotationsPerSecond.of(22.5))));
    controller.a().onFalse(new RunCommand(() -> intake._rollerIO.runVoltage(Volts.of(0.0)), intake));

    // Spindexer 1:1
    // controller.x().whileTrue(hopper.runSpindexer(18));
    // controller.x().onFalse(hopper.runSpindexer(0));

    // Tower - 15 Motor:7 Tower

    // controller.y().whileTrue(shooter.runTower(RotationsPerSecond.of(70)));
    // controller.y().onFalse(shooter.runTower(RotationsPerSecond.of(0)));

    // controller.b().onFalse(shooter.setHoodAngle(ShooterRotaryConstants.STARTING_ANGLE.magnitude()));
    // controller.povUp().onTrue(shooter.calibrateHood());
    // controller.povLeft().onTrue(shooter.setHoodAngle(10));
    // controller.povDown().onTrue(shooter.setHoodAngle(15));
    // controller.povRight().onTrue(shooter.setHoodAngle(20));

    // controller.x().onFalse(intake.setPivotAngle(IntakePivotConstants.STOW_ANGLE));
    controller.x().whileTrue(intake.setPivotAngle(IntakePivotConstants.PICKUP_ANGLE));
    controller.y().whileTrue(intake.setPivotAngle(IntakePivotConstants.STOW_ANGLE));

    controller.b().whileTrue(intake.zeroEncoder());

    // controller.povRight().onTrue(shooter.calibrateHood());

    controller.povLeft().whileTrue(new RunCommand(() -> climber._io.runPosition(Rotations.of(0.25), ClimberConstants.CRUISE_VELOCITY, ClimberConstants.ACCELERATION, ClimberConstants.JERK, PIDSlot.SLOT_1), climber));
    controller.povLeft().onFalse(climber.stopClimber());
    controller.povUp().whileTrue(climber.raiseClimber());
    controller.povUp().onFalse(climber.stopClimber());
    controller.povDown().whileTrue(climber.lowerClimber());
    controller.povDown().onFalse(climber.stopClimber());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
