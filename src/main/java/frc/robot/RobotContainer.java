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

import au.grapplerobotics.LaserCan;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.W8.io.absoluteencoder.AbsoluteEncoderIOCANCoderSim;
import frc.lib.W8.io.motor.*;
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
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.VisionConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterRotaryConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.BallCounter;
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
 private final Drive drive;
  private final Hopper hopper;
  private final Shooter shooter;
  private final Climber climber;
  private final Intake intake;
  private final BallCounter ballCounter;
  private final Vision vision;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if the robot is real before using the ball counter!
    if (Robot.isReal()) ballCounter = new BallCounter(new LaserCan(1));
    else ballCounter = null;

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
                        Ports.HopperRoller)));
        shooter =
            new Shooter(
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller)),
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller)),
                new RotaryMechanismReal(
                    new MotorIOTalonFX(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller),
                    Constants.ShooterRotaryConstants.CONSTANTS,
                    java.util.Optional.empty()));
        climber =
            new Climber(
                new LinearMechanismReal(
                    new MotorIOTalonFX(
                        ClimberConstants.MOTOR_NAME,
                        ClimberConstants.getFXConfig(),
                        Ports.ClimberLinearMechanism),
                    ClimberConstants.CHARACTERISTICS));

        intake =
            new Intake(
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        IntakeConstants.MOTOR_NAME,
                        IntakeConstants.getFXConfig(),
                        Ports.IntakeRoller)),
                new RotaryMechanismReal(
                    new MotorIOTalonFX(
                        IntakeConstants.MOTOR_NAME,
                        IntakeConstants.getFXConfig(),
                        Ports.IntakeRoller),
                    IntakeConstants.CONSTANTS,
                    Optional.of(
                        new AbsoluteEncoderIOCANCoderSim(
                            Ports.IntakeRoller,
                            IntakeConstants.MOTOR_NAME + " Encoder",
                            IntakeConstants.getCANcoderConfig(false)))));
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
                        HopperConstants.MOTOR_NAME,
                        HopperConstants.getFXConfig(),
                        Ports.HopperRoller),
                    HopperConstants.DCMOTOR,
                    HopperConstants.MOI,
                    HopperConstants.TOLERANCE));
        shooter =
            new Shooter(
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller),
                    ShooterFlywheelConstants.DCMOTOR,
                    ShooterFlywheelConstants.MOI,
                    ShooterFlywheelConstants.TOLERANCE),
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller),
                    ShooterFlywheelConstants.DCMOTOR,
                    ShooterFlywheelConstants.MOI,
                    ShooterFlywheelConstants.TOLERANCE),
                new RotaryMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterFlywheelConstants.NAME,
                        ShooterFlywheelConstants.getFXConfig(),
                        Ports.ShooterRoller),
                    ShooterRotaryConstants.DCMOTOR,
                    ShooterRotaryConstants.MOI,
                    true,
                    ShooterRotaryConstants.CONSTANTS,
                    java.util.Optional.empty()));
        climber =
            new Climber(
                new LinearMechanismSim(
                    new MotorIOTalonFXSim(
                        ClimberConstants.MOTOR_NAME,
                        ClimberConstants.getFXConfig(),
                        Ports.ClimberLinearMechanism),
                    ClimberConstants.DCMOTOR,
                    ClimberConstants.CARRIAGE_MASS,
                    ClimberConstants.CHARACTERISTICS,
                    true));

        intake =
            new Intake(
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        IntakeConstants.MOTOR_NAME,
                        IntakeConstants.getFXConfig(),
                        Ports.IntakeRoller),
                    IntakeConstants.DCMOTOR,
                    IntakeConstants.MOI,
                    IntakeConstants.TOLERANCE),
                new RotaryMechanismSim(
                    new MotorIOTalonFXSim(
                        IntakeConstants.MOTOR_NAME,
                        IntakeConstants.getFXConfig(),
                        Ports.IntakeRoller),
                    IntakeConstants.DCMOTOR,
                    IntakeConstants.MOI,
                    false,
                    IntakeConstants.CONSTANTS,
                    Optional.of(
                        new AbsoluteEncoderIOCANCoderSim(
                            Ports.IntakeRoller,
                            IntakeConstants.MOTOR_NAME + " Encoder",
                            IntakeConstants.getCANcoderConfig(false)))));
        vision =
            new Vision(
                new VisionIOPhotonVisionSim(
                    () -> drive.getPose(),
                    VisionConstants.camera0Name,
                    VisionConstants.robotToCamera0,
                    VisionConstants.aprilTagLayout,
                    PoseStrategy.CONSTRAINED_SOLVEPNP,
                    VisionConstants.getSystemSim()));
        break;

      default:
        //Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        hopper = new Hopper(new FlywheelMechanism() {});
        climber =
            new Climber(
                new LinearMechanism(
                    ClimberConstants.MOTOR_NAME, ClimberConstants.CHARACTERISTICS) {});

        shooter =
            new Shooter(
                new FlywheelMechanism() {},
                new FlywheelMechanism() {},
                new RotaryMechanism(null, null) {});

        intake =
            new Intake(
                new FlywheelMechanism() {},
                new RotaryMechanism(IntakeConstants.MOTOR_NAME, IntakeConstants.CONSTANTS) {});
        vision = new Vision(null);
        break;
    }

    // Extends climber arm
    NamedCommands.registerCommand("ExtendClimber", climber.runClimber(ClimberConstants.L1));
    // Retracts climber arm
    NamedCommands.registerCommand("Climb", climber.runClimber(ClimberConstants.STOW));
    // Bring flywheel up to speed + hood to position for known locations?
    NamedCommands.registerCommand("ReadyUp", getAutonomousCommand());
    // Shoots
    NamedCommands.registerCommand("Shoot", getAutonomousCommand());
    // Pulls in fuel - sets intake mode to 
    NamedCommands.registerCommand("IntakeOn", getAutonomousCommand());
    // Turns intake off - sets intake mode to  
    NamedCommands.registerCommand("IntakeOff", getAutonomousCommand());
    // Extends the intake
    NamedCommands.registerCommand("IntakeDown", getAutonomousCommand());

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

    //Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));

    //Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .x()
        .onTrue(Commands.runOnce(() -> hopper.setGoal(HopperConstants.HOPPER_POSITION), hopper));
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
