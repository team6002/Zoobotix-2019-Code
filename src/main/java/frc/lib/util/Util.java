/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.util;

/**
 * Commonly used functions
 */
public class Util {
    public static final double kEpsilon = 1e-12;


    private Util(){
        //prevent class from being instantiated
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude){
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max){
        return Math.min(max, Math.max(min,v));
    }

    public static boolean isAlmostEqual(double a, double b, double epsilon){
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean isAlmostEqual(double a, double b){
        return isAlmostEqual(a, b, kEpsilon);
    }

    public static boolean isAlmostEqual(int a, int b, int epsilon){
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
}
