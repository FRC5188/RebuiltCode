/*
 * Copyright (C) 2025 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.W8.io.motor;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.W8.util.Device.CAN;

/**
 * Abstraction for a simulated CTRE TalonFX motor implementing the {@link MotorIOSim} interface.
 * Wraps motor setup, control modes, telemetry polling, and error handling.
 *
 * <p>Because Phoenix TalonFX simulation does not simulate closed-loop PID control, this class
 * implements manual PID controllers for position and velocity modes. When runPosition() or
 * runVelocity() are called, the appropriate PID controller is enabled and its output voltage is
 * applied to the simulated motor via setInputVoltage().
 */
public class MotorIOTalonFXSim extends MotorIOTalonFX implements MotorIOSim {

  // Simulation configuration
  private static final double POSITION_KP = 50.0; // Position control proportional gain
  private static final double POSITION_KD = 5.0; // Position control derivative gain
  private static final double VELOCITY_KP = 0.1; // Velocity control proportional gain
  private static final double VELOCITY_KD = 0.0; // Velocity control derivative gain

  private double rotorToSensorRatio;
  private double sensorToMechanismRatio;
  private TalonFXSimState simState;

  // Manual PID control for simulation
  private final PIDController positionController = new PIDController(POSITION_KP, 0, POSITION_KD);
  private final PIDController velocityController = new PIDController(VELOCITY_KP, 0, VELOCITY_KD);

  // Track which control mode is active for manual PID
  private boolean positionClosedLoop = false;
  private boolean velocityClosedLoop = false;
  private double appliedVoltage = 0.0;

  /**
   * Constructs and initializes a TalonFX motor simulation.
   *
   * @param name The name of the motor(s)
   * @param config Configuration to apply to the motor(s)
   * @param main CAN ID of the main motor
   * @param followerData Configuration data for the follower(s)
   */
  public MotorIOTalonFXSim(
      String name, TalonFXConfiguration config, CAN main, TalonFXFollower... followerData) {
    super(name, config, main, followerData);

    rotorToSensorRatio = config.Feedback.RotorToSensorRatio;
    sensorToMechanismRatio = config.Feedback.SensorToMechanismRatio;
    simState = super.motor.getSimState();
  }

  @Override
  public void setPosition(Angle position) {
    simState.setRawRotorPosition(position.times(rotorToSensorRatio * sensorToMechanismRatio));
  }

  @Override
  public void setVelocity(AngularVelocity velocity) {
    simState.setRotorVelocity(velocity.times(rotorToSensorRatio * sensorToMechanismRatio));
  }

  @Override
  public void setRotorAcceleration(AngularAcceleration acceleration) {
    simState.setRotorAcceleration(acceleration);
  }

  @Override
  public double getRotorToSensorRatio() {
    return rotorToSensorRatio;
  }

  @Override
  public double getSensorToMechanismRatio() {
    return sensorToMechanismRatio;
  }

  @Override
  public void setEncoderPosition(Angle position) {
    super.setEncoderPosition(position.times(rotorToSensorRatio * sensorToMechanismRatio));
  }

  /**
   * Runs the motor to a target position using manual PID control (since Phoenix sim doesn't
   * simulate closed-loop control).
   */
  @Override
  public void runPosition(
      Angle position,
      AngularVelocity cruiseVelocity,
      AngularAcceleration acceleration,
      Velocity<AngularAccelerationUnit> maxJerk,
      PIDSlot slot) {
    this.goalPosition = position;
    double newSetpoint = position.in(Rotations);

    // Always reset PID controller when a new position command is issued
    // This ensures fresh control for each new setpoint
    positionController.reset();

    positionClosedLoop = true;
    velocityClosedLoop = false;
    positionController.setSetpoint(newSetpoint);
  }

  /**
   * Runs the motor at a target velocity using manual PID control (since Phoenix sim doesn't
   * simulate closed-loop control).
   */
  @Override
  public void runVelocity(
      AngularVelocity velocity, AngularAcceleration acceleration, PIDSlot slot) {
    double newSetpoint = velocity.in(RotationsPerSecond);

    // Always reset PID controller when a new velocity command is issued
    // This ensures fresh control for each new setpoint
    velocityController.reset();

    velocityClosedLoop = true;
    positionClosedLoop = false;
    velocityController.setSetpoint(newSetpoint);
  }

  @Override
  public void updateInputs(MotorInputs inputs) {
    // Run manual PID control for closed-loop modes since Phoenix sim doesn't simulate firmware
    if (positionClosedLoop) {
      double currentPosition = super.position.getValue().in(Rotations);
      double pidOutput = positionController.calculate(currentPosition);
      appliedVoltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
      // Apply calculated voltage via VoltageOut control
      super.motor.setControl(super.voltageControl.withOutput(appliedVoltage));
    } else if (velocityClosedLoop) {
      double currentVelocity = super.velocity.getValue().in(RotationsPerSecond);
      double pidOutput = velocityController.calculate(currentVelocity);
      appliedVoltage = MathUtil.clamp(pidOutput, -12.0, 12.0);
      // Apply calculated voltage via VoltageOut control
      super.motor.setControl(super.voltageControl.withOutput(appliedVoltage));
    } else {
      appliedVoltage = 0.0;
      positionController.reset();
      velocityController.reset();
    }

    inputs.connected =
        BaseStatusSignal.refreshAll(
                super.position,
                super.velocity,
                super.supplyVoltage,
                super.supplyCurrent,
                super.torqueCurrent,
                super.temperature,
                super.closedLoopError,
                super.closedLoopReference,
                super.closedLoopReferenceSlope)
            .isOK();

    simState.setSupplyVoltage(RobotController.getBatteryVoltage());

    inputs.position = super.position.getValue();
    inputs.velocity = super.velocity.getValue();
    // Use the voltage we calculated, not what the simState reports (which may not have updated yet)
    inputs.appliedVoltage = Volts.of(appliedVoltage);
    inputs.supplyCurrent = simState.getSupplyCurrentMeasure();
    inputs.torqueCurrent = simState.getTorqueCurrentMeasure();
    inputs.temperature = super.temperature.getValue();

    // Interpret control-loop status signals conditionally based on current mode
    Double closedLoopErrorValue = super.closedLoopError.getValue();
    Double closedLoopTargetValue = super.closedLoopReference.getValue();

    boolean isRunningPositionControl = super.isRunningPositionControl();
    boolean isRunningMotionMagic = super.isRunningMotionMagic();
    boolean isRunningVelocityControl = super.isRunningVelocityControl();

    inputs.positionError = isRunningPositionControl ? Rotations.of(closedLoopErrorValue) : null;

    inputs.activeTrajectoryPosition =
        isRunningPositionControl && isRunningMotionMagic
            ? Rotations.of(closedLoopTargetValue)
            : null;

    inputs.goalPosition = isRunningPositionControl ? goalPosition : null;

    if (isRunningVelocityControl) {
      inputs.velocityError = RotationsPerSecond.of(closedLoopErrorValue);
      inputs.activeTrajectoryVelocity = RotationsPerSecond.of(closedLoopTargetValue);
    } else if (isRunningPositionControl && isRunningMotionMagic) {
      var targetVelocity = closedLoopReferenceSlope.getValue();
      inputs.velocityError =
          RotationsPerSecond.of(targetVelocity - inputs.velocity.in(RotationsPerSecond));
      inputs.activeTrajectoryVelocity = RotationsPerSecond.of(targetVelocity);
    } else {
      inputs.velocityError = null;
      inputs.activeTrajectoryVelocity = null;
    }

    inputs.controlType = super.getCurrentControlType();
  }

  @Override
  public void close() {
    super.motor.close();
  }
}
