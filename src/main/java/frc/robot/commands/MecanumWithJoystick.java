// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.Math;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.MecanumSubsystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;


import frc.robot.DriveMathJoystick;
import frc.robot.DriveMathJoystickMecanum;

/** An example command that uses an example subsystem. */
public class MecanumWithJoystick extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final MecanumSubsystem mecanumSubsystem;
  private final Joystick joystick = RobotContainer.joystick;


  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  ShuffleboardTab driver;
  //ADIS16470_IMU gyro;
  //NetworkTableEntry kPEntry;
  double sensitivity = 0.82;
  //int targetRotation;
  boolean reversing;
  

  public MecanumWithJoystick(MecanumSubsystem mecanumSubsystem) {
    this.mecanumSubsystem = mecanumSubsystem;
    //gyro
    //this.gyro = gyro;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mecanumSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("The joystick drivetrain command thing is working! This is good.");
    driver =Shuffleboard.getTab("robot_things");
    UsbCamera camera = CameraServer.startAutomaticCapture();
    driver.add(camera).withSize(6,6);
    reversing = false;
    
    //Gyro in shuffleboard
    //driver.add(gyro).withSize(2, 2);
    //driver.addNumber("Gyro Angle", gyro::getAngle).withSize(3,3);


  }

  
  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //for broken joystick, gety, getx * -1
    //go when the joystick goes. all the math is for the sensitivity toggle (the slider thing on joystick) there's probably a more efficient way to do the same math but i do not care.

    

    //sensitivity diagnostics lol
    //for reference of future coders, this is a very nice way to display a number without looking at it on the riolog awkwardly.
   /* SmartDashboard.putNumber("Joystick Y Axis Input", joystick.getY());
    SmartDashboard.putNumber("Joystick X Axis Input", joystick.getX());
    SmartDashboard.putNumber("Joystick Sensitivity Input", joystick.getRawAxis(Constants.JoystickConstants.sliderAxis));
    SmartDashboard.putNumber("Current Speed Output", DriveMathJoystick.calculateSpeed(joystick.getX(), joystick.getRawAxis(3)));
*/
    

    //if (RobotContainer.joystick.getRawButton(5)) driveTrainSubsystem.armBrake.set(0.5);
   // if (RobotContainer.joystick.getRawButton(3)) driveTrainSubsystem.armBrake.set(-0.5);
    if (RobotContainer.joystick.getRawButton(7)){
      sensitivity = 0.5;
    }
    if (RobotContainer.joystick.getRawButton(8)){
      sensitivity = 0.66;
    }
    if (RobotContainer.joystick.getRawButton(9)){
      sensitivity = 0.82;
    }
    if (RobotContainer.joystick.getRawButton(10)){
      sensitivity = 1;
    }
    /* 
    mecanumSubsystem.m_leftDrive.set(joystick.getY());
    mecanumSubsystem.m_leftDriveTwo.set(joystick.getY());
    mecanumSubsystem.m_rightDrive.set(joystick.getY());
    mecanumSubsystem.m_rightDriveTwo.set(joystick.getY());
    */
    //System.out.println(DriveMathJoystickMecanum.calculateTurnSpeed(joystick.getZ(), sensitivity));
    mecanumSubsystem.mecanumDrive.driveCartesian(DriveMathJoystickMecanum.calculateSpeed(joystick.getY(), joystick.getRawAxis(3)), DriveMathJoystickMecanum.calculateSpeed((joystick.getX() * -1), joystick.getRawAxis(3)), DriveMathJoystickMecanum.calculateTurnSpeed(joystick.getZ(), sensitivity));

    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
