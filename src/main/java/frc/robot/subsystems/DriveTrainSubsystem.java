// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrainSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final PWMSparkMax m_leftDrive = Constants.ConstantMotors.m_leftDrive;
  private final PWMSparkMax m_leftDriveTwo = Constants.ConstantMotors.m_leftDriveTwo;
  private final MotorControllerGroup m_leftDriveGroup = new MotorControllerGroup(m_leftDrive, m_leftDriveTwo);
  private final PWMSparkMax m_rightDrive = Constants.ConstantMotors.m_rightDrive;
  private final PWMSparkMax m_rightDriveTwo = Constants.ConstantMotors.m_rightDriveTwo;
  private final MotorControllerGroup m_rightDriveGroup = new MotorControllerGroup(m_rightDrive, m_rightDriveTwo);
  public final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDriveGroup, m_rightDriveGroup);
 //this stuff should probably be in an arm subsystem but who has time for that
  public final PWMSparkMax armMover = Constants.ConstantMotors.armMover;
  public final PWMSparkMax armOpener = Constants.ConstantMotors.armOpener;
  public final PWMSparkMax armBrake = Constants.ConstantMotors.armBrake;
  public boolean brakeOpen = false;



  public DriveTrainSubsystem() {
   /* m_rightDriveGroup.setInverted(false);
    m_leftDriveGroup.setInverted(true);*/
  }

  @Override
  public void periodic() {
    
  }

}
