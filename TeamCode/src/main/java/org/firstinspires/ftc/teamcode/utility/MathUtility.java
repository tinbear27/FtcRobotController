package org.firstinspires.ftc.teamcode.utility;

public class MathUtility {
    public static double clamp(double num, double min, double max) {
        return Math.max(min, Math.min(num, max));
    }


}
