package frc.lib.W8.io.motor;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.W8.util.Device;
import java.util.EnumMap;

/**
 * Abstraction for a REV motor implementing the {@link MotorIO} interface. Wraps motor setup,
 * control modes.
 */
public class MotorIORev implements MotorIO {

  public SparkBase motor;
  public SparkClosedLoopController controller;
  public RelativeEncoder encoder;

  private final String name;

  /**
   * REV WORKAROUNDS
   *
   * <p>The motorIO interface was made for CTRE hardware. Some code does not map 1:1 to REV. The
   * below are workarounds to make the mapping easier.
   */

  /** Maps MotorIO.PIDSlot to REV ClosedLoopSlot. */
  private static final EnumMap<MotorIO.PIDSlot, ClosedLoopSlot> SLOT_MAP =
      new EnumMap<>(MotorIO.PIDSlot.class);

  static {
    SLOT_MAP.put(MotorIO.PIDSlot.SLOT_0, ClosedLoopSlot.kSlot0);
    SLOT_MAP.put(MotorIO.PIDSlot.SLOT_1, ClosedLoopSlot.kSlot1);
    SLOT_MAP.put(MotorIO.PIDSlot.SLOT_2, ClosedLoopSlot.kSlot2);
  }

  /**
   * Gets the corresponding REV ClosedLoopSlot for a given MotorIO.PIDSlot.
   *
   * @param slot The MotorIO.PIDSlot to convert.
   */
  protected ClosedLoopSlot getClosedLoopSlot(MotorIO.PIDSlot slot) {
    return SLOT_MAP.getOrDefault(slot, ClosedLoopSlot.kSlot0);
  }

  /** Maps MotorIO.ControlType to REV ControlType. */
  private static final EnumMap<ControlType, com.revrobotics.spark.SparkBase.ControlType>
      CONTROL_TYPE_MAP = new EnumMap<>(ControlType.class);

  static {
    CONTROL_TYPE_MAP.put(ControlType.VOLTAGE, com.revrobotics.spark.SparkBase.ControlType.kVoltage);
    CONTROL_TYPE_MAP.put(ControlType.CURRENT, com.revrobotics.spark.SparkBase.ControlType.kCurrent);
    CONTROL_TYPE_MAP.put(
        ControlType.DUTYCYCLE, com.revrobotics.spark.SparkBase.ControlType.kDutyCycle);
    CONTROL_TYPE_MAP.put(
        ControlType.POSITION, com.revrobotics.spark.SparkBase.ControlType.kPosition);
    CONTROL_TYPE_MAP.put(
        ControlType.VELOCITY, com.revrobotics.spark.SparkBase.ControlType.kVelocity);
  }

  /**
   * Gets the corresponding REV ControlType for a given MotorIO.ControlType.
   *
   * @param type The MotorIO.ControlType to convert.
   */
  protected com.revrobotics.spark.SparkBase.ControlType getRevControlType(ControlType type) {
    return CONTROL_TYPE_MAP.getOrDefault(
        type, com.revrobotics.spark.SparkBase.ControlType.kDutyCycle);
  }

  /**
   * Configuration data for a REV motor that follows another REV motor.
   *
   * <p>Follower motors mirror the output of a main motor, useful for mechanisms that require
   * multiple motors working together (like a dual-motor elevator).
   *
   * @param id The CAN device ID of the follower motor
   * @param opposesMain Whether this follower should spin opposite to the main motor
   */
  public record RevFollower(Device.CAN id, boolean opposesMain) {}

  /**
   * Constructs and initializes a REV motor.
   *
   * <p>This constructor applies the provided configuration to the main motor and all followers. It
   * sets up the follower relationship, initializes telemetry, and configures encoder. All followers
   * must be on the same CAN bus as the main motor.
   *
   * @param name The name of the motor(s) for logging and identification
   * @param CAN the CAN device reference containing the motor's CAN ID
   * @param isFlex True if using SparkFlex, false for SparkMax
   * @param config Configuration to apply to the motor(s) including PID, limits, and gear ratios.
   *     Note: pass though a SparkFlex or SparkMax config or else max motion will not behave as
   *     expected
   * @param followerData Configuration data for the follower motor(s), can be empty if no followers
   */
  @SuppressWarnings("resource")
  public MotorIORev(
      String name,
      Device.CAN CAN,
      boolean isFlex,
      SparkBaseConfig config,
      RevFollower... followerData) {
    this.name = name;

    // Initialize motor based on whether it's flex or not
    if (isFlex) {
      motor = new SparkFlex(CAN.id(), MotorType.kBrushless);
    } else {
      motor = new SparkMax(CAN.id(), MotorType.kBrushless);
    }

    // Applies config to the motor
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // Define controller and encoder
    controller = motor.getClosedLoopController();
    encoder = motor.getEncoder();

    // For loop that iterates for each follower mototr
    for (int i = 0; i < followerData.length; i++) {
      SparkBase motor_follower;
      SparkBaseConfig config_follower;

      // Initialize motor and new config based on whether it's flex or not
      if (isFlex) {
        motor_follower = new SparkFlex(followerData[i].id.id(), MotorType.kBrushless);
        config_follower = new SparkFlexConfig();
      } else {
        motor_follower = new SparkMax(followerData[i].id.id(), MotorType.kBrushless);
        config_follower = new SparkMaxConfig();
      }

      // Defines the config to follow main motor
      config_follower.apply(config).follow(CAN.id(), followerData[i].opposesMain);

      // Applies config to follower motor
      motor_follower.configure(
          config_follower, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
  }

  @Override
  public String getName() {
    return name;
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    inputs.connected = true;
    inputs.position = Rotation.of(encoder.getPosition());
    inputs.velocity = RotationsPerSecond.of(encoder.getVelocity());
    inputs.appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
    inputs.supplyCurrent = Amps.of(motor.getOutputCurrent());
    inputs.temperature = Celsius.of(motor.getMotorTemperature());
    inputs.positionError = null;
    inputs.positionError = null;
    inputs.velocityError = null;
    inputs.goalPosition = null;
    inputs.controlType = null;
    inputs.torqueCurrent = null;
  }

  @Override
  public void runBrake() {
    SparkBaseConfig newConfig = newEmptyConfig();
    newConfig.idleMode(IdleMode.kBrake);
    motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void runCoast() {
    SparkBaseConfig newConfig = newEmptyConfig();
    newConfig.idleMode(IdleMode.kCoast);
    motor.configure(newConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  @Override
  public void runVoltage(Voltage volts) {
    controller.setReference(volts.in(Volts), getRevControlType(ControlType.VOLTAGE));
  }

  @Override
  public void runCurrent(Current current) {
    controller.setReference(current.in(Amps), getRevControlType(ControlType.CURRENT));
  }

  @Override
  public void runDutyCycle(double percent) {
    motor.set(percent);
  }

  @Override
  public void runPosition(
      Angle position,
      AngularVelocity cruiseVelocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> maxJerk,
      PIDSlot slot) {
    controller.setReference(
        position.in(Rotation), getRevControlType(ControlType.POSITION), getClosedLoopSlot(slot));
  }

  @Override
  public void runVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, PIDSlot slot) {
    controller.setReference(
        velocity.in(RotationsPerSecond),
        getRevControlType(ControlType.VELOCITY),
        getClosedLoopSlot(slot));
  }

  @Override
  public void setEncoderPosition(Angle position) {
    encoder.setPosition(position.in(Rotation));
  }

  private SparkBaseConfig newEmptyConfig() {
    return (motor instanceof SparkFlex) ? new SparkFlexConfig() : new SparkMaxConfig();
  }

  protected void close() {
    this.motor.close();
  }

  protected SparkBase getMotor() {
    return this.motor;
  }
}
