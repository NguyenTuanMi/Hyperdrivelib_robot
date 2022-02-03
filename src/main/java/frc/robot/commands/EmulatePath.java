// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.hyperdrive.emulation.IEmulateParams;
import frc.robot.util.hyperdrive.emulation.PreferenceEmulationParams;
import frc.robot.util.hyperdrive.emulation.TankTrajectory;
import frc.robot.subsystems.DriveBase;
import frc.robot.util.hyperdrive.util.Path;
import frc.robot.util.hyperdrive.util.Units;
import static frc.robot.Constants.ROBOT_DATA.*;

public class EmulatePath extends CommandBase {
    private DriveBase drivetrain;
  /** Creates a new EmulatePath. */
  public EmulatePath(DriveBase drivebase){
    this.drivetrain = drivebase;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      //create parameters and load path
    IEmulateParams emulateParameters = new PreferenceEmulationParams(drivetrain.gethyperdrive().getLengthUnits());
    //drivetrain.gethyperdrive().loadPath(drivetrain.gethyperdrive().getRecordedPath(), emulateParameters);
    drivetrain.gethyperdrive().loadPath(new Path("file_directory"),emulateParameters); //- Order the robot to follow the given file path
    //perform calculations vital to Hyperdrive's performance
    drivetrain.gethyperdrive().performInitialCalculations();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      //below method is chainable. See the class and method reference in the manual
    //for methods to modify the trajectory, such as invertDirection() or invertTurn().
    TankTrajectory newTrajectory = drivetrain.gethyperdrive() 
    .calculateNextMovements()
    .getTankTrajectory(ROBOT_WHEELBASE_WIDTH)
    .convertTime(Units.TIME.DECASECONDS); // Return the trajectory imformation that can drive tank-drive robot

    double
    newLeftOutput = newTrajectory.getLeftPercentOutput(drivetrain.getCurrentVelocity(drivetrain.leftMaster)),
    newRightOutput = newTrajectory.getRightPercentOutput(drivetrain.getCurrentVelocity(drivetrain.rightMaster));

//this line will change depending on how your drivetrain is written, but you get the idea.
    drivetrain.drive(newLeftOutput, newRightOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.gethyperdrive().finishPath();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivetrain.gethyperdrive().pathFinished();
  }
}
