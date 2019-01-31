/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.RobotMap;
import frc.robot.commands.DriveTrainCommand;

/**
 * Add your docs here.
 */
public class DriveTrainSubSystem extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  public Talon  leftMotor;
  public Talon  rightMotor;
  public DifferentialDrive  drive;

  public DriveTrainSubSystem() {
    leftMotor = new Talon(RobotMap.leftMotor);
    rightMotor = new Talon(RobotMap.rightMotor);
    drive = new DifferentialDrive(leftMotor, rightMotor);
  }

  public void driveRobot(double leftSpeed, double rightSpeed) {
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveTrainCommand());
  }
}
