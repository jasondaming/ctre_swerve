// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;

public class Arm extends TrapezoidProfileSubsystem {
  private static final int armPrimaryID = 1;
  private static final int armFollowerID = 2;
  private CANSparkMax m_motor;
  private CANSparkMax m_follower;
  private SparkPIDController m_pidController;
  private SparkAbsoluteEncoder m_absoluteEncoder;
  private ArmFeedforward m_armFF;

  // Arm setpoints in  rotations
  private static final double intakePosition = 0.0;
  private static final double shootPosition = 0.0;
  private static final double ampPosition = 0.0;

  // Arm Contraints
  private static final double kMaxVelocityRadPerSecond = 0.0;
  private static final double kMaxAccelerationRadPerSecSquared = 0.0;
  private static final double kArmOffsetRads = 0.0;

  /** Creates a new Arm. */
  public Arm() {
    super(
      new TrapezoidProfile.Constraints(
          kMaxVelocityRadPerSecond, kMaxAccelerationRadPerSecSquared),
      kArmOffsetRads);

    // initialize motor
    m_motor = new CANSparkMax(armPrimaryID, MotorType.kBrushless);
    m_follower = new CANSparkMax(armFollowerID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_follower.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_follower.setInverted(true);
    m_motor.setIdleMode(IdleMode.kBrake);
    m_follower.setIdleMode(IdleMode.kBrake);
    m_follower.follow(m_motor);

    m_absoluteEncoder = m_motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    m_pidController = m_motor.getPIDController();
    m_pidController.setP(0.1);
    m_pidController.setI(0);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.25, 0.25);
    m_pidController.setFeedbackDevice(m_absoluteEncoder);

    m_motor.burnFlash();

    m_armFF = new ArmFeedforward(0, 0.1, 0);
  }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_armFF.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output
    m_pidController.setReference(setpoint.position, ControlType.kPosition, 0, feedforward / 12.0);
  }

  public Command setIntakePosition() {
    return setArmGoalCommand(intakePosition);
  }

  public Command setShootPosition() {
    return setArmGoalCommand(shootPosition);
  }

  public Command setAmpPosition() {
    return setArmGoalCommand(ampPosition);
  }

  public Command setArmGoalCommand(double kArmOffsetRads) {
    return Commands.runOnce(() -> setGoal(kArmOffsetRads), this);
  }
}
