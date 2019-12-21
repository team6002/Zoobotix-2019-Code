/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.io.Console;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotMap;
// import frc.robot.Logging;
import frc.robot.Constants;
import frc.robot.loops.*;

/**
 * Add your docs here.
 */
public class Drive extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  CANSparkMax mLeftMaster = new CANSparkMax(Constants.kLeftMasterPort, MotorType.kBrushless);
  CANSparkMax mRightMaster = new CANSparkMax(Constants.kRightMasterPort, MotorType.kBrushless);
  CANSparkMax mLeftSlave = new CANSparkMax(Constants.kLeftSlavePort, MotorType.kBrushless);
  CANSparkMax mRightSlave = new CANSparkMax(Constants.kRightSlavePort, MotorType.kBrushless);

  // WPI_TalonSRX mLeftMaster = new WPI_TalonSRX(Constants.kLeftMasterPort);
  // WPI_TalonSRX mRightMaster = new WPI_TalonSRX(Constants.kRightMasterPort);
  // VictorSPX mLeftSlave = new VictorSPX(Constants.kLeftSlavePort);
  // VictorSPX mRightSlave = new VictorSPX(Constants.kRightSlavePort);
  Solenoid mShifter = new Solenoid(Constants.kShifter);
  Compressor mCompressor = new Compressor(0);

  DifferentialDrive mDriveTrain = new DifferentialDrive(mLeftMaster, mRightMaster);

  private AHRS mNavx;
  private double mMaxVelocity=0;
  private double mVelocity=0;
  private double mTotalDistance=0;
  private double mTotalTravelTime=0;
  private double mDecelerateTime=0;
  private double mTargetAngle=0;
  private double mCorrection=0;
  private boolean mPathStarted=false;
  private boolean mPathCompleted=false;
  private boolean mDriveForward=true;
  private boolean mDriveStraight=true;
  private boolean mHighGear = false;

  private final double kDegreePerInch=360/(3.14*6);//19.108
  private static final double kAcceleration=700;//needs testing 

  private Timer runtime = new Timer();

  public enum DriveState{
    OPEN_LOOP, //voltage
    PATH_FOLLOWING //pid driving
  }
  private DriveState mDriveState = DriveState.OPEN_LOOP;
  
  private static Drive mInstance = null; 
  public synchronized static Drive getInstance(){
    if (mInstance == null) {
      mInstance = new Drive();
    }
    return mInstance;
  }

  

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timestamp) {
        synchronized (Drive.this) {
            // setOpenLoop(0, 0);
            setBrakeMode(false);
            // setVelocitySetpoint(0, 0);
            mNavx.reset();
        }
    }

    @Override
    public void onLoop(double timestamp) {
        synchronized (Drive.this) {
          switch (mDriveState) {
            case OPEN_LOOP:
              return;
            case PATH_FOLLOWING:
              // updatePathFollower();
              return;
            default:
                System.out.println("Unexpected drive control state: " + mDriveState);
                break;
          }    
        }
    }

    @Override
    public void onStop(double timestamp) {
        stop();

    }
  };

  
  public void registerEnabledLoops(ILooper in) {
    in.register(mLoop);
  }

  private Drive(){
    mCompressor.setClosedLoopControl(true);

    // mLeftMaster.configFactoryDefault();
    // mLeftSlave.configFactoryDefault();
    // mRightMaster.configFactoryDefault();
    // mRightSlave.configFactoryDefault();

    // mLeftSlave.follow(mLeftMaster);
    // mRightSlave.follow(mRightMaster);

    // mLeftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveTimeoutMs);
    // mLeftMaster.setSensorPhase(true);
    // mLeftMaster.setInverted(true);
    // mLeftSlave.setInverted(InvertType.FollowMaster);
    // mRightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kDriveTimeoutMs);
    // mRightMaster.setSensorPhase(true);
    // mRightMaster.setInverted(false);
    // mRightSlave.setInverted(InvertType.FollowMaster);

    //SPARK NEO
    mLeftMaster.restoreFactoryDefaults();
    mRightMaster.restoreFactoryDefaults();
    mLeftSlave.restoreFactoryDefaults();
    mRightSlave.restoreFactoryDefaults();

    mLeftSlave.follow(mLeftMaster, false);
    mRightSlave.follow(mRightMaster, false);

    setBrakeMode(false);//true for brake, false for coast.

    mLeftMaster.setInverted(true);
    mRightMaster.setInverted(false);
    mLeftMaster.setOpenLoopRampRate(0.1);
    mRightMaster.setOpenLoopRampRate(0.1);
    

    // mLeftMaster.configNominalOutputForward(0, Constants.kDriveTimeoutMs);
    // mLeftMaster.configNominalOutputReverse(0, Constants.kDriveTimeoutMs);
    // mLeftMaster.configPeakOutputForward(1, Constants.kDriveTimeoutMs);
    // mLeftMaster.configPeakOutputReverse(-1, Constants.kDriveTimeoutMs);
   
    // mRightMaster.configNominalOutputForward(0, Constants.kDriveTimeoutMs);
    // mRightMaster.configNominalOutputReverse(0, Constants.kDriveTimeoutMs);
    // mRightMaster.configPeakOutputForward(1, Constants.kDriveTimeoutMs);
    // mRightMaster.configPeakOutputReverse(-1, Constants.kDriveTimeoutMs); 
   
    // mLeftMaster.config_kF(Constants.kVelocityControlSlot, Constants.kDriveVelocityKf, 0);
    // mLeftMaster.config_kP(Constants.kVelocityControlSlot, Constants.kDriveVelocityKp, 0);
    // mLeftMaster.config_kI(Constants.kVelocityControlSlot, Constants.kDriveVelocityKi, 0);
    // mLeftMaster.config_kD(Constants.kVelocityControlSlot, Constants.kDriveVelocityKd, 0);
    
    // mLeftMaster.config_IntegralZone(Constants.kVelocityControlSlot, Constants.kDriveVelocityIZone, 0);
    // mLeftMaster.configClosedloopRamp(Constants.kDriveVelocityRampRate, 0);
        
    // mRightMaster.config_kF(Constants.kVelocityControlSlot, Constants.kDriveVelocityKf, 0);
    // mRightMaster.config_kP(Constants.kVelocityControlSlot, Constants.kDriveVelocityKp, 0);
    // mRightMaster.config_kI(Constants.kVelocityControlSlot, Constants.kDriveVelocityKi, 0);
    // mRightMaster.config_kD(Constants.kVelocityControlSlot, Constants.kDriveVelocityKd, 0);
    
    // mRightMaster.config_IntegralZone(Constants.kVelocityControlSlot, Constants.kDriveVelocityIZone, 0);
    // mRightMaster.configClosedloopRamp(Constants.kDriveVelocityRampRate, 0);
    
    mDriveTrain.setRightSideInverted(false);

    mNavx = new AHRS(SPI.Port.kMXP);
    mMaxVelocity=0;
    mVelocity=0;
    mTotalDistance=0;
    mTotalTravelTime=0;
    mDecelerateTime=0;
  }
  
  public void shift(){
    mShifter.set(!mHighGear);
    mHighGear = !mHighGear;
  }
  //CHEESY DRIVE USING WPILIB INTEGRATION
  public void setCurvatureDrive(double y, double x, boolean quickTurn){
    mDriveTrain.curvatureDrive(y, x, quickTurn);
  }

  public void setOpenLoop(double pLeftPower, double pRightPower){
    if (mDriveState != DriveState.OPEN_LOOP){
      setDriveMotor(0,0);
      mDriveState = DriveState.OPEN_LOOP;
    } 
    setDriveMotor(pLeftPower, pRightPower);
    // Logging.consoleLog(" speed: "  + getVelocity() + " out: " + getMotorOutput());
  }

  // public void setOpenLoopVelocity(double joy) {
  //   if (mDriveState != DriveState.OPEN_LOOP){
  //     setDriveMotor(0,0);
  //     mDriveState = DriveState.OPEN_LOOP;
  //   }
  //   double speed = 2100 * joy; // 2100 rpm in either direction
  //   mLeftMaster.set(ControlMode.Velocity, speed);
  //   mRightMaster.set(ControlMode.Velocity, speed);

  //   // Logging.consoleLog(" speed: " + getVelocity() + " out: " + getMotorOutput() + " target: " + speed
  //   //                     + " error: " + mLeftMaster.getClosedLoopError());

  // }

  private void setDriveMotor(double pLeftPower, double pRightPower){
    mLeftMaster.set(pLeftPower);
    mRightMaster.set(pRightPower);
  }

  
  

  // public void updatePathFollower() {
  //   if (mDriveState == DriveState.PATH_FOLLOWING) {
  //     if (!mPathStarted) {
  //       mPathStarted = true;
  //       runtime.reset();
  //       // return;   
  //     }

  //       calculateSpeed((runtime.get()));
  //      // calculateCorrection();
  //       setSpeed();
  //   } 
  //   else { // Not following a path
     
       
  //   }
  //  }

  //  private void calculateCorrection() {
  //   double angle = mNavx.getAngle();
  //   double error = Math.abs(mTargetAngle - angle);
  //   mCorrection = (error * 17);
  //   if (mTargetAngle < angle) mCorrection= - mCorrection;
  //   // Logging.consoleLog("mDrive.calculateCorrection Target=%.2f Current=%.2f degree Correct=%.2f", mTargetAngle, angle,mCorrection);
  //  }

    // private void setSpeed() {
    //   double speed = mVelocity;
    //   if (!mDriveForward) speed = -speed; // going backward? reverse speed sign
    //   if (mDriveStraight) { // drive forward or backwrad
    //     mLeftMaster.set(ControlMode.Velocity, mVelocity);
    //     mRightMaster.set(ControlMode.Velocity, mVelocity);
    //   } else { // turning
    //     // m_LeftFrontWheel.setPower(speed);
    //     // m_LeftBackWheel.setPower(speed);
    //     // m_RightFrontWheel.setPower(-speed);
    //     // m_RightBackWheel.setPower(-speed);
    //     }
    //   }

  //     public void driveForward(double p_Inches) {
  //       double p_Angle = mNavx.getAngle();
  //       driveForward(p_Inches, p_Angle);
        
  //       // Logging.consoleLog("mDrive.driveForward %.1f inches at %.2f degree", p_Inches, p_Angle);
  //     }
      
  //   public void driveForward(double p_Inches, double p_Angle) {
  //       System.out.println("mDrive.driveForward " + p_Inches + " inches at " + p_Angle + " degree");
  //       // Logging.consoleLog("mDrive.driveForward %.1f inches at %.2f degree", p_Inches, p_Angle);
  //       // PIDFCoefficients pid = mLeftMaster.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
  //       // RobotLog.d("HSR- SUB_drive.driveForward p=%.2f i%.2f d%.2f f%.2f", pid.p, pid.i, pid.d, pid.f);
  
  //      mTargetAngle = p_Angle;
  //      mDriveStraight = true;
       
  //      if (p_Inches < 0) {
  //        mDriveForward = false;
  //      } else {
  //        mDriveForward = true;
  //      }
  //      p_Inches = Math.abs(p_Inches);
  //      if (p_Inches < 4) mTotalDistance = p_Inches * 1.5;
  //      else mTotalDistance = p_Inches;

  //      calculatePathParameters(mTotalDistance);
  //      mPathCompleted = false;
  //      mPathStarted = false;
  //      mDriveState = DriveState.PATH_FOLLOWING;
  //    }

  //   int loop = 0; 
  //   public double calculateSpeed(double pSeconds){
  //     double pTime=pSeconds; // current moment in run 
  //     if (pTime >= mTotalTravelTime) { // end of Run
  //       mDriveState = DriveState.OPEN_LOOP;
  //       mPathCompleted = true;
  //       mVelocity =0;
  //       return mVelocity;
  //     } else if (pTime > mDecelerateTime) { // decelerating
  //       pTime = (mTotalTravelTime - pSeconds);
  //     }
      
  //     mVelocity = kAcceleration*pTime;
  //     // if(loop < 2){
  //     //   loop++;
  //     // }
  //     // else {
  //       // Logging.consoleLog(" Target: " + ((double)((int)mVelocity*10))/10 + " Current: "  + getVelocity() + " Error: " + mLeftMaster.getClosedLoopError() + " Time: " + pTime);
  //       // loop = 0;
  //     // }

  //     return mVelocity;
  //   }

  //   public boolean pathCompleted(){
  //     // updatePathFollower();
  //     return mPathCompleted;
  //   }

  // public void calculatePathParameters(double pTotalDistance){
  //   double totalDegree =(pTotalDistance*kDegreePerInch);
  //   mMaxVelocity = Math.sqrt(kAcceleration * totalDegree);
  //   mTotalTravelTime = totalDegree/(mMaxVelocity/2);
  //   mDecelerateTime = mTotalTravelTime/2;
  // }
  // public double getTotalTravelTime(){//used to calculate how much error will be accumulated
  //   return mTotalTravelTime;
  // }

  private void setBrakeMode(boolean pBrake){
    if(pBrake == true){
      // mLeftMaster.setNeutralMode(NeutralMode.Brake);
      // mRightMaster.setNeutralMode(NeutralMode.Brake);
      // mLeftSlave.setNeutralMode(NeutralMode.Brake);
      // mRightSlave.setNeutralMode(NeutralMode.Brake);
      mLeftMaster.setIdleMode(IdleMode.kBrake);
      mRightMaster.setIdleMode(IdleMode.kBrake);
      mLeftSlave.setIdleMode(IdleMode.kBrake);
      mRightSlave.setIdleMode(IdleMode.kBrake);
    }
    else {
      // mLeftMaster.setNeutralMode(NeutralMode.Coast);
      // mRightMaster.setNeutralMode(NeutralMode.Coast);
      // mLeftSlave.setNeutralMode(NeutralMode.Coast);
      // mRightSlave.setNeutralMode(NeutralMode.Coast);
      mLeftMaster.setIdleMode(IdleMode.kCoast);
      mRightMaster.setIdleMode(IdleMode.kCoast);
      mLeftSlave.setIdleMode(IdleMode.kCoast);
      mRightSlave.setIdleMode(IdleMode.kCoast);
    }
  }

  public void updateTelemetry(){
    // SmartDashboard.putNumber("Motor velocity", getVelocity());
    // SmartDashboard.putNumber("mVelocity", mVelocity);
    SmartDashboard.putBoolean("High Gear", mHighGear);
    SmartDashboard.putBoolean("Compressor Status", mCompressor.enabled());
  }
  // public double getVelocity(){
  //   return mLeftMaster.getEncoder().getVelocity();
  // }

  public double getTarget(){
    return ((double)((int)mVelocity*10))/10;
  }

  public double getAngle(){
    return mNavx.getAngle();
  }

  private void stop(){
    mDriveState = DriveState.OPEN_LOOP;
    setOpenLoop(0, 0);

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
