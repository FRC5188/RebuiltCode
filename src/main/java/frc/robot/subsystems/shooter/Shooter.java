// package frc.robot.subsystems.shooter;

// import static edu.wpi.first.units.Units.RotationsPerSecond;

// import edu.wpi.first.units.Units;
// import edu.wpi.first.units.measure.AngularVelocity;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.W8.io.motor.MotorIO.PIDSlot;
// import frc.lib.W8.mechanisms.flywheel.FlywheelMechanism;
// import frc.robot.Constants.FeederConstants;
// import frc.robot.Constants.ShooterConstants;

// public class Shooter extends SubsystemBase {

//   private FlywheelMechanism _flywheel;
//   private FlywheelMechanism _feeder;
//   public double desiredVelo;

//   public Shooter(FlywheelMechanism flywheel, FlywheelMechanism feeder) {
//     _flywheel = flywheel;
//     _feeder = feeder;
//   }

//   // Sets feeder motor speed
//   public void runFeeder() {
//     _feeder.runVelocity(
//         FeederConstants.FEED_SPEED, FeederConstants.FEED_ACCELERATION, PIDSlot.SLOT_2);
//   }

//   // Sets the flywheel velocity based on an input.
//   public void setFlywheelVelocity(double velocity) {
//     AngularVelocity angVelo = RotationsPerSecond.of(velocity);
//     velocity = desiredVelo;

//     _flywheel.runVelocity(angVelo, ShooterConstants.ACCELERATION, PIDSlot.SLOT_0);
//   }

//   public enum State {
//     OFF(Units.RevolutionsPerSecond.of(0.0)),
//     IDLE(Units.RevolutionsPerSecond.of(ShooterConstants.IDLE_SPEED_RPM / 60)),
//     SHOOT_FROM_HUB(Units.RevolutionsPerSecond.of(ShooterConstants.HUB_SPEED_RPM / 60)),
//     SHOOT_FROM_TOWER(Units.RevolutionsPerSecond.of(ShooterConstants.TOWER_SPEED_RPM / 60)),
//     SHOOT(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60)),
//     SHOOT_ON_MOVE(Units.RevolutionsPerSecond.of(ShooterConstants.DEFAULT_SPEED_RPM / 60));

//     private final AngularVelocity stateVelocity;

//     State(AngularVelocity stateVelocity) {
//       this.stateVelocity = stateVelocity;
//     }
//   }

//   // Checks if the flywheel is at speed and returns a boolean
//   public boolean flyAtVelocity() {
//     return Math.abs(desiredVelo - _flywheel.getVelocity().in(RotationsPerSecond))
//         <= ShooterConstants.FLYWHEEL_VELOCITY_TOLERANCE;
//   }

//   public Command shoot(double velocity) {
//     return Commands.run(
//             () -> {
//               setFlywheelVelocity(velocity);
//             })
//         .until(() -> flyAtVelocity())
//         .andThen(
//             () -> {
//               runFeeder();
//             })
//         .andThen(
//             () -> {
//               setFlywheelVelocity(0);
//             });
//   }

//   @Override
//   public void periodic() {}
// }
