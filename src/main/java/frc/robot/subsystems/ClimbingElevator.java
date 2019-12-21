/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.SuperstructureConstants;
// import frc.robot.Logging;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;
import frc.lib.util.Util;

/**
 * Subsystem for the elevator at the back of the robot that helps us climb 
 */
public class ClimbingElevator extends Subsystem {
  
  private CANSparkMax mClimbingElevatorLeader;
  private CANSparkMax mClimbingElevatorFollower;
  private CANPIDController mPIDC;
  private TalonSRX mClimbingElevatorWheel = new TalonSRX(Constants.kClimbingElevatorWheelPort);

  private double kP = Constants.kClimbingElevatorKp;
  private double kI = Constants.kClimbingElevatorKi;
  private double kD = Constants.kClimbingElevatorKd;
  private double kF = Constants.kClimbingElevatorKf;
  

  //forklift items considered part of climbing elevator... cause its for climbing.
  private Solenoid mForklift = new Solenoid(Constants.kForkliftPort);
  private Solenoid mForkliftLatch = new Solenoid(Constants.kForkliftLatchPort);
  private Solenoid mForkliftRatchet = new Solenoid(Constants.kForkliftRatchetPort);

  private double targetPos;

  private enum ControlState{
    OPEN_LOOP,
    MOTION_MAGIC,
    POSTION_PID
  }
  private ControlState mControlState = ControlState.OPEN_LOOP;

  public enum WantedState{
    IDLE,
    GO_TO_POSITION,
    WANT_MANUAL,
  }
  private WantedState mWantedState = WantedState.WANT_MANUAL;

  public enum SystemState{
    HOLDING_POSITION,
    MOVING_TO_POSITION,
    MANUAL,
  }
  private SystemState mSystemState = SystemState.MANUAL;

  private static ClimbingElevator mInstance = null; 
  public synchronized static ClimbingElevator getInstance(){
    if (mInstance == null) {
      mInstance = new ClimbingElevator();
    }
    return mInstance;
  }

  private final Loop mLoop = new Loop() {
    @Override
    public void onStart(double timestamp) {
        synchronized (ClimbingElevator.this) {
            
        }
    }

    @Override
    public void onLoop(double timestamp) {

        SystemState newState;
        synchronized (ClimbingElevator.this) {
          double p = SmartDashboard.getNumber("CEle P", 0);
          double i = SmartDashboard.getNumber("CEle I", 0);
          double d = SmartDashboard.getNumber("CEle D", 0);
          
          if(p != kP){
            kP = p;
            mPIDC.setP(kP);
          }
          if(i != kI){
            kI = i;
            mPIDC.setI(kI);
          }
          if(d != kD){
            kD = d;
            mPIDC.setD(kD);
          }
        

          // if(newState != mSystemState){
          //   System.out.println("Elevator State " + mSystemState + " to " + newState);
          //   // Logging.consoleLog("Elevator State " + mSystemState + " to " + newState);
          //   mSystemState = newState;
          // }
          // if(mControlState == ControlState.MOTION_MAGIC){
            // mClimbingElevator.set(ControlMode.MotionMagic, targetPos);
          // }

          }
    }

    @Override
    public void onStop(double timestamp) {
        stop();

    }
  };
  private SystemState handleHoldingPosition(){
    // mClimbingElevator.set(ControlMode.Position, getPosition());
    // setPosition(getPosition());

    switch(mWantedState){
      case IDLE:
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        return SystemState.HOLDING_POSITION;
    }
  }

  private SystemState handleMovingToPosition(){
    if(isFinished()){
      setWantedState(WantedState.IDLE);
      return SystemState.HOLDING_POSITION;
    }
    // setPosition(targetPos);
    // mElevator.set(ControlMode.MotionMagic, targetPos);

    switch(mWantedState){
      case IDLE:
        //hold elevator where it is
        // setPosition(getPosition());
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost Elevator state in: " + mSystemState);
        // Logging.consoleLog("Lost Elevator state in: " + mSystemState );
        return SystemState.HOLDING_POSITION;
    }
  }

  private SystemState handleManual(){
    switch(mWantedState){
      case IDLE:
        //stop elevator where it is
        // setPosition(getPosition());
        // mElevator.set(ControlMode.MotionMagic, targetPos);
        return SystemState.HOLDING_POSITION;
      case GO_TO_POSITION:
        return SystemState.MOVING_TO_POSITION;
      case WANT_MANUAL:
        return SystemState.MANUAL;
      default: 
        System.out.println("Lost Elevator state in: " + mSystemState);
        // Logging.consoleLog("Lost Elevator state in: " + mSystemState );
        return SystemState.MANUAL;
    }
  }
  
  public void registerEnabledLoops(ILooper in) {
    in.register(mLoop);
  }

  public ClimbingElevator(){
    mClimbingElevatorLeader = new CANSparkMax(Constants.kClimbingElevatorLeaderPort, MotorType.kBrushless);
    mClimbingElevatorFollower = new CANSparkMax(Constants.kClimbingElevatorFollowerPort, MotorType.kBrushless);

    mPIDC = mClimbingElevatorLeader.getPIDController();

    mClimbingElevatorLeader.restoreFactoryDefaults();
    mClimbingElevatorFollower.restoreFactoryDefaults();
    mClimbingElevatorWheel.configFactoryDefault();

    mClimbingElevatorFollower.follow(mClimbingElevatorLeader, true);

    mClimbingElevatorLeader.setIdleMode(IdleMode.kBrake);
    mClimbingElevatorFollower.setIdleMode(IdleMode.kBrake);

    mClimbingElevatorWheel.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, Constants.kTimeOut);
    mClimbingElevatorWheel.setSelectedSensorPosition(0);
    mClimbingElevatorWheel.setNeutralMode(NeutralMode.Coast);

    mPIDC.setP(kP);
    mPIDC.setI(kI);
    mPIDC.setD(kD);
    mPIDC.setFF(kF);

    mPIDC.setOutputRange(-1, 1);
    mPIDC.setSmartMotionMaxAccel(Constants.kClimbingElevatorAcceleration, Constants.smartMotionSlot);
    mPIDC.setSmartMotionMaxVelocity(Constants.kClimbingElevatorCrusingVelocity, Constants.smartMotionSlot);
    mPIDC.setSmartMotionAllowedClosedLoopError(0, Constants.smartMotionSlot);
    
    targetPos = getPosition();
  }

  private boolean deployedForklift = false;
  public void toggleForklift(){
    if(deployedForklift){
      mForklift.set(false);
      deployedForklift = false;
    }else{
      mForklift.set(true);
      deployedForklift = true;
    }
  }

  public void unlockForkliftLatch(){
    mForkliftLatch.set(true);
  }

  public void unlockForkliftRatchet(){
    mForkliftRatchet.set(true);
  }
  public void lockForkliftRatchet(){
    mForkliftRatchet.set(false);
  }

  public void setWheel(double power){
    mClimbingElevatorWheel.set(ControlMode.PercentOutput,power);
  }

  public void setOpenLoop(double percentage){
    if(mControlState != ControlState.OPEN_LOOP){
      mControlState = ControlState.OPEN_LOOP;
    }
    setWantedState(WantedState.WANT_MANUAL);
    mClimbingElevatorLeader.set(percentage);
  }

  public void setPosition(int pos){
    // if(mControlState != ControlState.MOTION_MAGIC){
    //   mControlState = ControlState.MOTION_MAGIC;
    // }
    int wanted = pos;
    // if(wanted < SuperstructureConstants.kClimbingElevatorMax){
    //   wanted = SuperstructureConstants.kClimbingElevatorMax;
    // }
    mPIDC.setReference(wanted, ControlType.kSmartMotion);
    // setWantedState(WantedState.GO_TO_POSITION);
  }

  public void stopElevator(){
    mPIDC.setReference(0, ControlType.kVoltage);//just set motor to not move.
  }

  public void setWantedState(WantedState wanted){
    mWantedState = wanted;
  }

  public boolean isFinished(){
    return mControlState == ControlState.MOTION_MAGIC && 
            Util.isAlmostEqual(mClimbingElevatorLeader.getEncoder().getPosition(), targetPos, 500);//within 500 ticks of target.
  }

  public double getPosition(){
    return mClimbingElevatorLeader.getEncoder().getPosition();
  }

  public int getWheelPosition(){
    return mClimbingElevatorWheel.getSelectedSensorPosition();
  }

  public void resetEncoder(){//only does this when the climbingElevator is at home position.
    mClimbingElevatorLeader.getEncoder().setPosition(0);
  }

  public void resetWheelEncoder(){
    mClimbingElevatorWheel.setSelectedSensorPosition(0);
  }

  public void stop(){
    setOpenLoop(0.0);
  }

  public void updateTelemetry(){
    SmartDashboard.putNumber("CElevator Pos", getPosition());
    SmartDashboard.putNumber("CElevator Volts", mClimbingElevatorLeader.getAppliedOutput());
    // SmartDashboard.putNumber("CElevatorWheel Pos", getWheelPosition());
    // SmartDashboard.putNumber("CElevator AbsPos", mClimbingElevator.getSensorCollection().getPulseWidthPosition());
    // SmartDashboard.putNumber("CElevator Output", mClimbingElevator.getOutputCurrent());
    // SmartDashboard.putBoolean("CE isFinished", isFinished());
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
