package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import java.lang.Math;

public class DriveMathJoystick {

    public static double calculateSpeed(double jy, double jz)
    {
        return jy * Math.abs(jy) * ( (jz * -0.25) + 0.75);
    }

    public static double calculateTurnSpeed(double jx, double sense)
    {
        return jx * Math.abs(jx) * sense;
    }
}