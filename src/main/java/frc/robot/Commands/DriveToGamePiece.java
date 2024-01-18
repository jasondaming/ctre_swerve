// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.Vision.Limelight;
import frc.robot.generated.TunerConstants;

/** Add your docs here. */
public class DriveToGamePiece extends Command {

  private Limelight ll;
  private CommandSwerveDrivetrain drivetrain;
  private PIDController thetaController = new PIDController(4.0, 0, 0.05);
  public DriveToGamePiece(CommandSwerveDrivetrain drivetrain, Limelight Limelight) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    ll = Limelight;
  }
  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDeadband(TunerConstants.kSpeedAt12VoltsMps * 0.01).withRotationalDeadband(Constants.Drive.MaxAngularRate * 0.01)
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private double thetaOutput = 0;
  private final double xOutput = 0.2; // Speed to drive towards note will increase after testing
  private final double yOutput = 0;
  private double setpoint = 0; // How far the camera is offset from the center in degrees

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    thetaController.reset();
    thetaController.setTolerance(0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
		if (ll.hasTarget()){
      setpoint = Math.toRadians(-ll.getNoteHorizontal())+ drivetrain.getState().Pose.getRotation().getRadians();
			thetaController.setSetpoint(setpoint);
      if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(drivetrain.getState().Pose.getRotation().getRadians(), setpoint);
			}
		}
    
    drivetrain.setControl(drive.withVelocityX(-xOutput * TunerConstants.kSpeedAt12VoltsMps).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}