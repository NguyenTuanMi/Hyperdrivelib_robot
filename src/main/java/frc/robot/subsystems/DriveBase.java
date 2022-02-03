// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.util.hyperdrive.Hyperdrive;
import frc.robot.util.hyperdrive.enumeration.DriveStyle;
import frc.robot.util.hyperdrive.util.Units;
import static frc.robot.Constants.ROBOT_DATA.*;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import frc.robot.subsystems.Gyro;


public class DriveBase extends SubsystemBase {
  public WPI_TalonSRX rightFollow = new WPI_TalonSRX(7);
  public WPI_TalonSRX leftFollow = new WPI_TalonSRX(8);
  public WPI_TalonSRX rightMaster = new WPI_TalonSRX(9);
  public WPI_TalonSRX leftMaster = new WPI_TalonSRX(10);
  public Gyro gyro = Gyro.getInstance();
  private Hyperdrive hyperdrive = new Hyperdrive(
    DriveStyle.TANK, 
    Units.LENGTH.METERS, 
    motorunit_per_unit , 
    Units.FORCE.KILOGRAM_FORCE,
    robot_weight); 
  

  /** Creates a new DriveBase_ver2. */
  public DriveBase() {
    
    leftMaster.setNeutralMode(NeutralMode.Brake);
    leftFollow.setNeutralMode(NeutralMode.Brake);
    rightFollow.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
  
    leftFollow.follow(leftMaster);
    rightFollow.follow(rightMaster);
  
    leftMaster.setInverted(false);
    leftFollow.setInverted(false);

    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
    
  }

  public Hyperdrive gethyperdrive() {
    return hyperdrive;
  }

  public void Zero() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
    gyro.setYaw();
    hyperdrive.zeroPositionAndHeading();
  }

  public double getCurrentVelocity (WPI_TalonSRX srx) {
    return srx.getSelectedSensorVelocity();
  }

  public void drive(double left_velocity, double right_velocity) {
    rightMaster.set(right_velocity);
    leftMaster.set(left_velocity);
  }

  @Override
  public void periodic() {
    double leftPosition = this.leftMaster.getSelectedSensorPosition();
    double rightPosition = this.rightMaster.getSelectedSensorPosition();
    double angle = gyro.getYaw();
    hyperdrive.update(leftPosition,rightPosition,angle);
    // This method will be called once per scheduler run
  }
}
