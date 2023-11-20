package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.DriveMathMecanum;
import frc.robot.RobotContainer;
import frc.robot.subsystems.MecanumSubsystem;

public class MecanumWithXBox extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final MecanumSubsystem mecanumSubsystem;
    private final XboxController xboxController = RobotContainer.xboxController;

    public MecanumWithXBox(MecanumSubsystem mecanumSubsystem) {
        this.mecanumSubsystem = mecanumSubsystem;
        //gyro
        //this.gyro = gyro;
        
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(mecanumSubsystem);
      }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        mecanumSubsystem.mecanumDrive.driveCartesian(
            DriveMathMecanum.calculateSpeed(xboxController.getLeftY(), 1-xboxController.getLeftTriggerAxis()), 
            DriveMathMecanum.calculateSpeed((xboxController.getLeftX() * -1), 1-xboxController.getLeftTriggerAxis()), 
            DriveMathMecanum.calculateTurnSpeed(xboxController.getRightX()*-1, 0.82)
        );

    }

}
