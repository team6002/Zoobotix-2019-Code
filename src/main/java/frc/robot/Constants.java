/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class Constants {
    public final static int kDriveTimeoutMs = 100;
    public final static int kTimeOut = 100;
    public final static double kLooperDt = 0.01;
    public final static int kProfileID0 = 0;
    public final static int kProfileID1 = 1;
    public final static int kPositionControlSlot = 0;
    public final static int kMotionMagicSlot = 1;

    //Solenoids
    public final static int kShifter = 0;
    public final static int kHatchBeak = 1;
    public final static int kHatchPlatform = 2;
    public final static int kJaw = 3;
    public final static int kForkliftPort = 4;
    public final static int kForkliftLatchPort = 5;
    public final static int kForkliftRatchetPort = 6;//the ratchet

    //drive motor ports NEEDS TO BE CHANGED TO SPARK MAXS! Comments are talon and victors.
    public final static int kLeftMasterPort = 1;
    public final static int kLeftSlavePort = 16;
    public final static int kRightMasterPort = 14;
    public final static int kRightSlavePort = 15;

    //Intake motor ports
    public final static int kIntakeArmPort = 2;
    public final static int kIntakeArmSlavePort = 9;
    public final static int kIntakePort = 4;

    //Climbing elevator ports
    // public final static int kClimbingElevatorPort = 6;
    public final static int kClimbingElevatorLeaderPort = 28;
    public final static int kClimbingElevatorFollowerPort = 27;

    public final static int kClimbingElevatorWheelPort = 7;

    //Elevator ports
    public final static int kElevatorPort = 12;
    public final static int kElevatorSlavePort = 3;
    

    //Claw ports
    public final static int kClawWrist = 10;
    public final static int kClawIntake = 13;

    //Arm ports
    public final static int kArmPort = 11;

    //DRIVE PID
    public final static double kDriveVelocityKf = 0.071358816964;
    public final static double kDriveVelocityKp = 2.8; //were at 2.6
    public final static double kDriveVelocityKi = 0; //was at .0065
    public final static double kDriveVelocityKd = 30; // ws at 18

    public final static int kDriveVelocityIZone = 0;
    public final static double kDriveVelocityRampRate = 0;
    public final static int kVelocityControlSlot = 0;

    public final static int kIntakeArmPIDLoopID0 = 0;
    public final static int kIntakeArmPIDLoopID1 = 1;

    //INTAKE ARM PID for positioning before climb
    public final static double kIntakeArmKf = 0;
    public final static double kIntakeArmKp = 5;
    public final static double kIntakeArmKi = 0.0;
    public final static double kIntakeArmKd = 0;

    public final static int kIntakeArmCruiseVelocity = 12000;
    public final static int kIntakeArmAcceleration = 12000;

    //INTAKE ARM LOAD PID for climbing
    public final static double kIntakeArmLoadKf = 0;
    public final static double kIntakeArmLoadKp = 3.0;
    public final static double kIntakeArmLoadKi = 0;
    public final static double kIntakeArmLoadKd = 25;

    public final static int kIntakeArmLoadCruiseVelocity = 30;
    public final static int kIntakeArmLoadAcceleration = 30;

    //CLIMBING ELEVATOR PID
    public final static int smartMotionSlot = 0;//for spark max smart motion.

    //Climbing elevator, needs i maybe?
    public final static double kClimbingElevatorKf = 0.00325;
    public final static double kClimbingElevatorKp = 5e-5;
    public final static double kClimbingElevatorKi = 0.00000001;
    public final static double kClimbingElevatorKd = 0.001;

    public final static int kClimbingElevatorCrusingVelocity = 300;
    public final static int kClimbingElevatorAcceleration = 300;

    //CLIMBING ELEVATOR WHEEL PID
    public final static double kClimbingElevatorWheelKf = 0;
    public final static double kClimbingElevatorWheelKp = 0.5;
    public final static double kClimbingElevatorWheelKi = 0;
    public final static double kClimbingElevatorWheelKd = 0;

    //ELEVATOR PID
    public final static double kElevatorKf = 0.05;//0.05;
    public final static double kElevatorKp = 0.8;
    public final static double kElevatorKi = 0;
    public final static double kElevatorKd = 0;

    public final static int kElevatorCruiseVelocity = 4000;
    public final static int kElevatorAcceleration = 4000;
    
    //CLAW WRIST PID
    public final static double kClawWristKf = 0;
    public final static double kClawWristKp = 1.8;
    public final static double kClawWristKi = 0.001;
    public final static double kClawWristKd = 0;

    public final static int kClawWristCruiseVelocity = 200;
    public final static int kClawWristAcceleration = 200;

    //ARM PID
    public final static double kArmKf = 0;
    public final static double kArmKp = 1.5;//1.75;
    public final static double kArmKi = 0.0000;
    public final static double kArmKd = 0;

    public final static double kArmClosedLoopRampRate = 0.15;

    public final static int kArmCruiseVelocity = 400;
    public final static int kArmAcceleration = 400;

    
    //offsets
    public static final int kArmOffset = 385;//for comp909;//
    public static final int kClawOffset = 1237;///for comp1497;//
    public static final int kIntakeArmOffset = 2865;//for comp787;//
}
