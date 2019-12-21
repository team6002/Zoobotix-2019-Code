/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Used by the superstructure to tell the subsystems where to go.
 * Holds positional values for each subsystem.
 */
public class SuperstructureConstants {

    //constraints for positional subsystems.
    //also positions for the elevator since we don't care about inbetween, only min and max.
    public static final int kElevatorMin = 0;
    public static final int kElevatorMax = 28500;


    //Climbing Elevator we gucci for spark maxs
    public static final int kClimbingElevatorMin = -2;
    public static final int kClimbingElevatorLevelTwo = -117;
    public static final int kClimbingElevatorLevelThree = -260;
    public static final int kClimbingElevatorMax = -260;

    //Climbing Elevator Wheel
    public static final int kClimbingElevatorWheelForward = 8000;

    public static final int kArmMinHeight = 0;
    public static final int kArmMaxHeight = 0;

    //safety checks for Intake
    public static final int kArmSafeForIntake = 1750;

    //positions for the main arm.
    public static final int kArmHome = 0;
    public static final int kArmIntakeHatch = 2495;
    public static final int kArmIntake = 2400;
    public static final int kArmIntakeLoading = 1600;
    public static final int kArmStowed = 2550;

    public static final int kArmClimb = 2620;
    public static final int kArmCargoLowFront = 2150;

    public static final int kCargoLow = 0;
    public static final int kCargoShip = 525;
    public static final int kCargoHigh = 900;//same position for mid and high ball port
    // public static final int kHatchLow = 0;
    public static final int kHatchHigh = 475;//same position for mid and high hatch.


    //positions for the wrist of the claw on the main arm
    public static final int kWristIntake = 1050;
    public static final int kWristStowed = 700;
    public static final int kWristHatchIntake = 360;// also used for low hatch
    public static final int kWristHatch = 515;//for mid and high hatch

    public static final int kWristClimb = 410;

    public static final int kWristIntakeLoading = 1200;
    public static final int kWristCargo = 285;
    public static final int kWristCargoShip = 330;
    public static final int kWristCargoLow = 775;

    public static final int kWristCargoLowFront = 625;


    //positions for the intake arm.
    public static final int kIntakeStowed = 25;
    public static final int kIntakeDeployed = -1470;//for intaking
    public static final int kIntakeLowered = -1525;//get intake out of the way so arm can leave with ball safely
    public static final int kIntakePreparedToClimb = -650;//-490

    public static final int kIntakePreLevelThree = -1175;

    public static final int kIntakeLevelTwo = -1800;
    public static final int kIntakeLevelTwoHalf = -1789;
    public static final int kIntakeLevelTwoHalfPlus = -1800;
    public static final int kIntakeClimb = -1825;
    
    
    
}
