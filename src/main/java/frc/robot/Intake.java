// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private static final int intakePrimaryID = 1;
  private CANSparkMax m_motor;
  private SparkPIDController m_pidController;

  // Intake Speed in RPM
  private static final double intakeSpeed = 50.0;

  /** Creates a new Arm. */
  public Intake() {
    // initialize motor
    m_motor = new CANSparkMax(intakePrimaryID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();
    m_motor.setInverted(false);
    m_motor.setIdleMode(IdleMode.kBrake);

    m_pidController = m_motor.getPIDController();
    m_pidController.setP(0.1);
    m_pidController.setI(0);
    m_pidController.setD(0);
    m_pidController.setIZone(0);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-0.25, 0.25);

    m_motor.burnFlash();
  }

  public Command intakeOn() {
    return this.runOnce(() -> m_pidController.setReference(intakeSpeed, ControlType.kVelocity));
  }

  public Command intakeOff() {
    return this.runOnce(() -> m_pidController.setReference(0, ControlType.kVelocity));
  }
}
