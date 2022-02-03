// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.controller.PIDController;
import frc.robot.subsystems.Gyro;
import static frc.robot.Constants.PID_Rotation.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.DriveBase;

public class RotateToAngle extends CommandBase {
  private PIDController pid = new PIDController(kp, ki, kd);
  private Gyro gyro;
  public double angleDeg;
  public DriveBase drive = new DriveBase();
  /** Creates a new RotateToAngle. */
  public RotateToAngle(double angle, DriveBase drivebase) {
    drive    = drivebase;
    angleDeg = angle;
    gyro     = Gyro.getInstance();
    pid.setTolerance(tolerance,angle_v_error);
    pid.setSetpoint(angleDeg);
    pid.enableContinuousInput(angle_minimumInput, angle_maximumInput);
    pid.setIntegratorRange(angle_minimumIntegral, angle_maximumIntegral);
    addRequirements(gyro);
    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro.reset();
    SmartDashboard.putNumber("Set point",angleDeg);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("Start", true);
    double velocity = pid.calculate(gyro.getYaw());
    velocity        = Math.min(abs_velocity,Math.min(-abs_velocity, velocity));
    drive.drive(-velocity, velocity);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
