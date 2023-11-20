package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import java.lang.Math;

public class DriveMathXbox {

    public static double calculateSpeed(double jy, double speediness)
    {
        return jy * Math.abs(jy) * speediness;
    }

    public static double calculateTurnSpeed(double jx, double turningSensitivity)
    {
        return jx * Math.abs(jx) * turningSensitivity;
    }
}