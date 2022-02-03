// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import com.kauailabs.navx.frc.AHRS;
import static frc.robot.Constants.Collision_Detection.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CollisionDetection extends CommandBase {
  private AHRS ahrs;
  private Timer timer;
  private double begin_accel_x;
  private double begin_accel_y;
  private boolean collision_state = false;
  /** Creates a new CollisionDetection. */
  public CollisionDetection() {
    try{
      ahrs = new AHRS();
    }
    catch (RuntimeException ex ) {
        DriverStation.reportError("Error instantiating Gyro:  " + ex.getMessage(), true);
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(time_elapsed)){
      double current_accel_x = ahrs.getWorldLinearAccelX();
      double current_accel_y = ahrs.getWorldLinearAccelY();
      double jerk_x = current_accel_x - begin_accel_x;
      double jerk_y = current_accel_y - begin_accel_y;
      if (Math.abs(jerk_x)/time_elapsed > k_collision || Math.abs(jerk_y)/time_elapsed > k_collision){
        SmartDashboard.putBoolean("Collision state", true);
        collision_state = true;
      }
      else {
        SmartDashboard.putBoolean("Collision state", false);
        begin_accel_x = current_accel_x;
        begin_accel_y = current_accel_y;
        current_accel_x = ahrs.getWorldLinearAccelX();
        current_accel_y = ahrs.getWorldLinearAccelY();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return collision_state;
  }
}
