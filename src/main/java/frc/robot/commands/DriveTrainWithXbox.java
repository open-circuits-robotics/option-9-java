package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveMathXbox;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveTrainWithXbox extends CommandBase
{


    //@SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"});
    private final DriveTrainSubsystem driveTrainSubsystem;
    private final XboxController xboxController;

    public double speediness = 1.0;
    public double turningSensitivity = 1.0;

    public DriveTrainWithXbox(DriveTrainSubsystem dTrainSubsystem)
    {
        driveTrainSubsystem = dTrainSubsystem;
        xboxController = RobotContainer.xboxController.getHID();
        addRequirements(dTrainSubsystem);
    }

    @Override
    public void initialize()
    {        
    }

    @Override
    public void execute()
    {
        if (xboxController.getLeftBumper() && speediness < 1.0) speediness = speediness + 0.01;
        if (xboxController.getRightBumper() && turningSensitivity < 1.0) turningSensitivity = turningSensitivity + 0.01;
        if (xboxController.getLeftBumper() && speediness > 0.5) speediness = speediness - 0.01;
        if (xboxController.getRightBumper() && turningSensitivity > 0.5) turningSensitivity = turningSensitivity - 0.01;
        this.driveTrainSubsystem.m_robotDrive.arcadeDrive(DriveMathXbox.calculateSpeed(xboxController.getLeftY(), speediness), DriveMathXbox.calculateTurnSpeed(xboxController.getLeftX(), turningSensitivity));

    }
}
