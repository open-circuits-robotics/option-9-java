package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class MecanumSubsystem extends SubsystemBase{
    public final PWMSparkMax m_leftDrive = Constants.ConstantMotors.m_leftDrive;
    public final PWMSparkMax m_leftDriveTwo = Constants.ConstantMotors.m_leftDriveTwo;
    public final PWMSparkMax m_rightDrive = Constants.ConstantMotors.m_rightDrive;
    public final PWMSparkMax m_rightDriveTwo = Constants.ConstantMotors.m_rightDriveTwo;
    public final MecanumDrive mecanumDrive = new MecanumDrive(m_leftDrive,m_leftDriveTwo,m_rightDrive,m_rightDriveTwo);

    public MecanumSubsystem(){
        m_leftDrive.setInverted(true); 
        m_leftDriveTwo.setInverted(true);
        m_rightDrive.setInverted(false);
        m_rightDriveTwo.setInverted(false);

    }

}