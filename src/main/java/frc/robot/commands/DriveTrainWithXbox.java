package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveMath;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainWithXbox extends CommandBase
{

    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"});
    private final DriveTrainSubsystem driveTrainSubsystem;
    private final XboxController xboxController;

    public DriveTrainWithXbox(DriveTrainSubsystem dTrainSubsystem)
    {
        driveTrainSubsystem = dTrainSubsystem;
        xboxController = RobotContainer.xboxController;
        addRequirements(dTrainSubsystem);
    }

    @Override
    public void initialize()
    {        
    }

    @Override
    public void execute()
    {
        
        this.driveTrainSubsystem.m_robotDrive.arcadeDrive(DriveMath.calculateSpeed(xboxController.getLeftY(), xboxController.getLeftTriggerAxis()), DriveMath.calculateTurnSpeed(xboxController.getLeftX(), xboxController.getRightTriggerAxis()));

    }
}
