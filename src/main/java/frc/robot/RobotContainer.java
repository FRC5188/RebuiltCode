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

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static frc.robot.subsystems.vision.VisionConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.W8.io.motor.*;
import frc.lib.W8.mechanisms.flywheel.*;
import frc.lib.W8.mechanisms.rotary.RotaryMechanism;
import frc.lib.W8.mechanisms.rotary.RotaryMechanismReal;
import frc.lib.W8.mechanisms.rotary.RotaryMechanismSim;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeFlywheelConstants;
import frc.robot.Constants.IntakePivotConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterFlywheelConstants;
import frc.robot.Constants.ShooterRotaryConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.multisubsystem_commands.CmdShootOnTheMove;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOLimelight;
import java.util.Optional;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  public final Drive drive;
  public final Hopper hopper;
  private final Shooter shooter;
  public final Intake intake;
  //   private final BallCounter ballCounter;
  private final Vision vision;
  private final Climber climber;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final GenericHID buttonbox1 = new GenericHID(1);
  private final GenericHID buttonbox2 = new GenericHID(2);

  private final JoystickButton zeroDriveButton = new JoystickButton(buttonbox1, 1);
  private final JoystickButton zeroIntakeButton = new JoystickButton(buttonbox1, 2);
  private final JoystickButton zeroHoodButton = new JoystickButton(buttonbox1, 3);
  private final JoystickButton climberDownButton = new JoystickButton(buttonbox1, 7);
  private final JoystickButton climberUpButton = new JoystickButton(buttonbox1, 8);
  private final JoystickButton zeroClimberButton = new JoystickButton(buttonbox1, 4);
  private final JoystickButton incrementHoodButton = new JoystickButton(buttonbox1, 5);
  private final JoystickButton decrementHoodButton = new JoystickButton(buttonbox1, 6);
  private final JoystickButton extendClimberButton = new JoystickButton(buttonbox2, 2);
  private final JoystickButton retractClimberButton = new JoystickButton(buttonbox2, 1);  

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Check if the robot is real before using the ball counter!
    // if (Robot.isReal()) ballCounter = new BallCounter(new LaserCan(1));
    // else ballCounter = null;

    // DISABLED IN SIM: BaseStatusSignal.setUpdateFrequencyForAll(50.0) was causing NT flooding
    // because it sets ALL status signals to 50Hz before the module IO can set specific frequencies
    // Let subsystem-specific configs handle signal frequencies instead
    if (Constants.currentMode == Constants.Mode.REAL) {
      BaseStatusSignal.setUpdateFrequencyForAll(50.0);
    }

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
                        Ports.RightFlywheel,
                        ShooterFlywheelConstants.FOLLOWER_1)),
                new FlywheelMechanismReal(
                    new MotorIOTalonFX(
                        FeederConstants.NAME,
                        FeederConstants.getFXConfig(false),
                        Ports.TowerRoller)),
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
                drive::addVisionMeasurement,
                new VisionIOLimelight("limelight", drive::getRotation));

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
                        Ports.RightFlywheel),
                    ShooterFlywheelConstants.DCMOTOR,
                    ShooterFlywheelConstants.MOI,
                    ShooterFlywheelConstants.TOLERANCE),
                new FlywheelMechanismSim(
                    new MotorIOTalonFXSim(
                        FeederConstants.NAME,
                        FeederConstants.getFXConfig(false),
                        Ports.TowerRoller),
                    FeederConstants.DCMOTOR,
                    FeederConstants.MOI,
                    FeederConstants.TOLERANCE),
                new RotaryMechanismSim(
                    new MotorIOTalonFXSim(
                        ShooterRotaryConstants.NAME,
                        ShooterRotaryConstants.getFXConfig(),
                        Ports.HoodMotor),
                    ShooterRotaryConstants.DCMOTOR,
                    ShooterRotaryConstants.MOI,
                    false,
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
                        Ports.IntakePivot),
                    IntakePivotConstants.DCMOTOR,
                    IntakePivotConstants.MOI,
                    false,
                    IntakePivotConstants.CONSTANTS,
                    Optional.empty()));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

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
    // LEAVE THIS UP HERE
    // Extends climber arm
    // NamedCommands.registerCommand("ExtendClimber", getAutonomousCommand());
    // // Retracts climber arm
    // NamedCommands.registerCommand("Climb", getAutonomousCommand());

    // Below Hub
    NamedCommands.registerCommand(
        "SetHeading45", DriveCommands.setHeading(drive, -3 * Math.PI / 4));
    // Aligned with Hub
    NamedCommands.registerCommand("SetHeading90", DriveCommands.setHeading(drive, 0));
    // Above Hub
    NamedCommands.registerCommand(
        "SetHeadingNeg45", DriveCommands.setHeading(drive, 3 * Math.PI / 4));

    // calibrate shooter
    NamedCommands.registerCommand("ZeroHood", shooter.calibrateHood());

    NamedCommands.registerCommand("TowerHood", shooter.setHoodAngle(7.5));

    // Calibrates the hood
    NamedCommands.registerCommand("ZeroHood", shooter.calibrateHood());

    // Runs tower, flywheel, spindexer, and auto hood angle
    NamedCommands.registerCommand("Shoot", Commands.parallel(
                shooter.score(),
                hopper.runSpindexer(RotationsPerSecond.of(14)),
                shooter.setAngleForDistance(() -> drive.getRadiusToHubInMeters())));
    // Stops tower, flywheels, and spindexer
    NamedCommands.registerCommand("Return", Commands.parallel(
                shooter.stopScore(),
                hopper.runSpindexer(RotationsPerSecond.of(0))));

    // Runs rollers and pickup position ONLY when called
    // This is a zoned command.
    NamedCommands.registerCommand("Intake", intake.intake().andThen(() -> intake.stop()));
    // Stops rollers - we shouldn't need this, but throw it in if autos are tweaking.
    NamedCommands.registerCommand("IntakeOff", Commands.run(() -> intake.stop()));

    NamedCommands.registerCommand("Jostle", Commands.run(() -> intake.setPivotAngle(IntakePivotConstants.JOSTLE_ANGLE)));
    
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addDefaultOption("Select An Auto", Commands.none());

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Just Shooter Flywheels
    controller.leftBumper().whileTrue(shooter.setVelocityForDistance(() -> drive.getRadiusToHubInMeters()));
    controller.leftBumper().onFalse(shooter.runFlywheel(RotationsPerSecond.of(0)));

    // Shoot
    controller
        .leftTrigger()
        .whileTrue(
            Commands.parallel(
                shooter.score(), hopper.runSpindexer(RotationsPerSecond.of(15)), intake.intake()));
    controller
        .leftTrigger()
        .onFalse(
            Commands.parallel(
                shooter.stopScore(),
                hopper.runSpindexer(RotationsPerSecond.of(0)),
                Commands.run(() -> intake.stop())));

    // Intake Rollers 11 Motor: 9 Intake

    // controller.a().whileTrue((intake.runRollers(RotationsPerSecond.of(22.5))));
    // controller
    //     .a()
    //     .onFalse(new RunCommand(() -> intake._rollerIO.runVoltage(Volts.of(0.0)), intake));

    // Intake + Out
    controller.rightTrigger().whileTrue(intake.intake());
    controller.rightTrigger().onFalse(Commands.runOnce(() -> intake.stop()));

    // Auto Align
    controller
        .a()
        .whileTrue(new CmdShootOnTheMove(
            drive, 
            shooter, 
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX()));

    // Jostle
    controller.b().onTrue(intake.jostleIntake());

    // Stow Intake
    controller.x().whileTrue(intake.setPivotAngle(IntakePivotConstants.STOW_ANGLE));

    // Fixed Shots
    controller.povUp().onTrue(shooter.setHoodAngle(2.3));
    controller.povRight().onTrue(shooter.setHoodAngle(9.8));
    controller.povDown().onTrue(shooter.setHoodAngle(15)); //8

    // controller.povUp().onTrue(shooter.incrementHoodAngle());
    // controller.povDown().onTrue(shooter.decrementHoodAngle());

    controller
        .povLeft()
        .onTrue((shooter.setAngleForDistance(() -> drive.getRadiusToHubInMeters())));

    // controller.povDown().onTrue(shooter.setAngleForDistance(Meters.of(1.0)));
    // controller.povRight().onTrue(shooter.setAngleForDistance(Meters.of(2.0)));
    // controller.povUp().onTrue(shooter.setAngleForDistance(Meters.of(5.0)));
    // Reset Buttons

    zeroDriveButton.onTrue(DriveCommands.zeroHeading(drive));
    zeroIntakeButton.onTrue(intake.zeroEncoder());
    zeroHoodButton.onTrue(shooter.calibrateHood());

    climberUpButton.onTrue(climber.raiseClimber());
    climberUpButton.onFalse(climber.stopClimber());
    climberDownButton.onTrue(climber.lowerClimber());
    climberDownButton.onFalse(climber.stopClimber());
    zeroClimberButton.onTrue(climber.calibrateClimber());

    incrementHoodButton.onTrue(shooter.incrementHoodAngle());
    decrementHoodButton.onTrue(shooter.decrementHoodAngle());

    extendClimberButton.onTrue(climber.runClimber());
    retractClimberButton.onTrue(climber.retractClimber());
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
