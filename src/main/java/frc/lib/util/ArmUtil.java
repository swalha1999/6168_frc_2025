package frc.lib.util;

public class ArmUtil {
    double x = 82.5; 
    double y = 0;

    public static double[] solveEA(double x, double y, double O, double RPH) {
        double sqrtTerm = Math.sqrt(-O * O + RPH * RPH - 2 * RPH * y + x * x + y * y);
        
        double a1 = 2 * Math.atan((x - sqrtTerm) / (O + RPH - y));        
        double e1 = (x - O * Math.sin(a1)) / Math.cos(a1);
        
        return new double[]{cmToEncoder(e1), radiansToEncoder(a1)};
    }

    public static double radiansToEncoder(double radians) {
        return (radians/1.274) *  10;
    }

    public static double cmToEncoder(double length) {
        return (length/180) * 74;
    }
    
}
