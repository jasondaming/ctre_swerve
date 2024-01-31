package frc.robot;


import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private final TalonFX m_shooterMotor = new TalonFX(0);
  private final TalonFX m_shooterMotor2 = new TalonFX(1);
  private final VelocityVoltage m_velocity = new VelocityVoltage(0);

  // The shooter subsystem for the robot.
  public Shooter() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelTalonConfig.MotorOutput.PeakReverseDutyCycle = 0; // Never go in reverse
    flywheelTalonConfig.Slot0.kP = 0.1;
    flywheelTalonConfig.Slot0.kD = 0;
    m_shooterMotor.getConfigurator().apply(flywheelTalonConfig);

    m_shooterMotor2.setControl(new Follower(0, true));
  }

  public double getShooterSpeed() {
    return m_shooterMotor.getVelocity().getValue();
  }

  public double getShooterVoltage() {
    return m_shooterMotor.getMotorVoltage().getValue();
  }

  public boolean atSetpoint() {
    return true;
  }

  public void setRPS(double rps) {
    m_shooterMotor.setControl(m_velocity.withVelocity(rps));
  }

  public Command setAutoSpeed() {
    return this.runOnce(() -> setRPS(10));
  }

  public Command setAmpSpeed() {
    return this.runOnce(() -> setRPS(1));
  }

  public Command setOffSpeed() {
    return this.runOnce(() -> setRPS(0));
  }

  public void relativeSpeedChange(double changeAmount) {
    m_shooterMotor.setControl(m_velocity.withVelocity(m_shooterMotor.getVelocity().getValue() + changeAmount));
  }
}