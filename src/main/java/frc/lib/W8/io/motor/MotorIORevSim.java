package frc.lib.W8.io.motor;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.SparkBaseConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import frc.lib.W8.util.Device;

/**
 * Simulated implementation of {@link MotorIORev} for REV Robotics motors using WPILib simulation.
 * Implements {@link MotorIOSim} to provide simulation-specific behavior.
 *
 * <p>This class wraps a simulated SparkFlex or SparkMax motor, allowing position and velocity
 * control in a simulation environment. It provides methods to run the motor in position or velocity
 * mode, update simulation inputs, and manage simulation state.
 *
 * @see MotorIORev
 * @see MotorIOSim
 */
public class MotorIORevSim extends MotorIORev implements MotorIOSim {

  public SparkBase motor;
  public SparkClosedLoopController controller;
  private SparkSim simState;
  private Time lastTime = Seconds.zero();

  private double RotorToSensorRatio;
  private double SensorToMechanismRatio;

  /**
   * Constructs a MotorIORevSim instance.
   *
   * @param name Name of the motor
   * @param CAN CAN device reference containing the motor's CAN ID
   * @param isFlex True if using SparkFlex, false for SparkMax
   * @param gearBox DCMotor gearbox model
   * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios.
   *     Note: pass though a SparkFlex or SparkMax config or else max motion will not behave as
   *     expected
   * @param followerData Configuration data for the follower motor(s), can be empty if no followers
   * @see MotorIO
   */
  public MotorIORevSim(
      String name,
      Device.CAN CAN,
      boolean isFlex,
      double RotorToSensorRatio,
      double SensorToMechanismRatio,
      DCMotor gearBox,
      SparkBaseConfig config,
      RevFollower... followerData) {
    super(name, CAN, isFlex, config, followerData);

    motor = this.getMotor();
    this.RotorToSensorRatio = RotorToSensorRatio;
    this.SensorToMechanismRatio = SensorToMechanismRatio;

    if (isFlex) {
      simState = new SparkFlexSim((SparkFlex) motor, gearBox);
    } else {
      simState = new SparkMaxSim((SparkMax) motor, gearBox);
    }

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    simState.enable();
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {

    simState.setBusVoltage(RoboRioSim.getVInVoltage());

    double deltaTime = Seconds.of(Timer.getTimestamp()).minus(lastTime).in(Seconds);

    simState.iterate(velocity.in(RotationsPerSecond), simState.getBusVoltage(), deltaTime);

    lastTime = Seconds.of(Timer.getTimestamp());
  }

  @Override
  public double getRotorToSensorRatio() {
    return RotorToSensorRatio;
  }

  public double getSensorToMechanismRatio() {
    return SensorToMechanismRatio;
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    inputs.position = Rotation.of(encoder.getPosition());
    inputs.velocity = RotationsPerSecond.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(simState.getAppliedOutput() * simState.getBusVoltage());
    inputs.supplyCurrent = Amps.of(simState.getMotorCurrent());
    inputs.temperature = Celsius.of(motor.getMotorTemperature());
  }

  @Override
  public void close() {
    super.motor.close();
  }
}
