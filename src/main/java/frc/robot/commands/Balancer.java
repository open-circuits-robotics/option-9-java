package frc.robot.commands;

import frc.robot.subsystems.DriveTrainSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class Balancer extends CommandBase {
    private DriveTrainSubsystem driveTrainSubsystem;
    public Balancer(DriveTrainSubsystem dtr){
        driveTrainSubsystem = dtr;
    }
    public boolean balance(){
        
        if (DriveTrainSubsystem.gyro.getYComplementaryAngle() > -15 && driveTrainSubsystem.gyro.getYComplementaryAngle() < 15)
        {
            return true;
        }
        if (driveTrainSubsystem.gyro.getYComplementaryAngle() > 0.0){
            driveTrainSubsystem.m_robotDrive.arcadeDrive(0.2,0.0);
        }
        if (driveTrainSubsystem.gyro.getYComplementaryAngle() < 0.0){
            driveTrainSubsystem.m_robotDrive.arcadeDrive(-0.2,0.0);
        }
        return false;
      
    }
    
}