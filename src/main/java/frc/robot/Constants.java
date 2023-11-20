// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ConstantMotors{
        public final static PWMSparkMax m_leftDrive = new PWMSparkMax(2);
        public final static PWMSparkMax m_leftDriveTwo = new PWMSparkMax(0);
        public final static PWMSparkMax m_rightDrive = new PWMSparkMax(3);
        public final static PWMSparkMax m_rightDriveTwo = new PWMSparkMax(1);
        public final static PWMSparkMax armMover = new PWMSparkMax(4);
        public final static PWMSparkMax armOpener = new PWMSparkMax(5);
        public final static PWMSparkMax armBrake = new PWMSparkMax(6); 
    }
    public static final class JoystickConstants{
        public static final int sliderAxis = 3;      // Raw Axis to change sensitivity
        public static final int raiseArmButton = 1;  // Raw Button to move arm up
        public static final int lowerArmButton = 2;  // Raw Button to move arm down
        public static final int openClawButton = 6;  // Raw Button to open claw
        public static final int closeClawButton = 4; // Raw Button to clone claw
    }
}
