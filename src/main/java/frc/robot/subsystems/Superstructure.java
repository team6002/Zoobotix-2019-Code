/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import frc.lib.util.Util;
import frc.robot.LatchedBoolean;
// import frc.robot.Logging;
import frc.robot.SuperstructureConstants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Looper;
import frc.robot.loops.Loop;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClimbingElevator;
import frc.robot.subsystems.IntakeArm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
/**
 * Used to tell the other subsystems what to do, and coordinate them together.
 */
public class Superstructure {
    private Elevator mElevator = Elevator.getInstance();
    private IntakeArm mIntakeArm = IntakeArm.getInstance();
    private Claw mClaw = Claw.getInstance();
    private ClimbingElevator mClimbingElevator = ClimbingElevator.getInstance();
    private Intake mIntake = Intake.getInstance();
    private Arm mArm = Arm.getInstance();

    public enum SuperstructureMode{ //Controlled by the Co-Driver so driver can focus on manuevers
        HATCH,
        CARGO,
        CLIMB,
    }
    private SuperstructureMode mSuperstructureMode = SuperstructureMode.HATCH;

    public enum WantedState {//codriver decides between cargo, hatch, and climb mode.
        IDLE,               //subsystem moves according to which mode is selected.

        GO_LOW,
        GO_MID,
        GO_HIGH,

        WANT_CAPTURE,//when the driver wants to secure a game piece.
        WANT_LOW_CAPTURE,// when driver wants to secure cargo in the low position
        PREPARE_CLIMB,
        WANT_CLIMB,//climb to level three!
        WANT_CLIMB_LEVEL_TWO,//climb to level two!
        PREPARE_INTAKE,

        //Operator controls for moving through state machine
        FORWARD_STATE,
        BACK_STATE,
        DEPLOY,//trigger button
        //switch between hatch mode and cargo mode 
        TOGGLE_HATCH_CARGO_MODE,
    }
    private WantedState mWantedState = WantedState.IDLE;

    public enum SystemState {


        PREPARING_FOR_CLIMB,
        CLIMBING,//robot is climbing with stages.
        LOWER_ROBOT,//reset robot from pre level three to prepare for climb


        STALL_CLIMB, //emergency stopped robot during climb, if one of our robots is falling off.

        //hatch systemstates
        HATCH_IDLE, //starting position.
        HATCH_PREPARE_FOR_INTAKE,
        HATCH_MID,
        HATCH_DEPLOYED_MID,
        HATCH_HIGH,
        HATCH_DEPLOYED_HIGH,
        HATCH_LOW,
        HATCH_DEPLOYED_LOW,
        //hatch moving states
        HATCH_PREPARE_FOR_INTAKE_TO_MID,
        HATCH_MID_TO_HIGH,
        HATCH_MID_TO_LOW,
        HATCH_HIGH_TO_MID,
        HATCH_LOW_TO_MID,
        HATCH_MID_TO_PREPARE_FOR_INTAKE,
        HATCH_HIGH_TO_PREPARE_FOR_INTAKE,
        HATCH_DEPLOYED_LOW_TO_PREPARE_FOR_INTAKE,
        HATCH_DEPLOYED_MID_TO_PREPARE_FOR_INTAKE,
        HATCH_DEPLOYED_HIGH_TO_PREPARE_FOR_INTAKE,
        HATCH_DEPLOYED_LOW_TO_PREPARE_CLIMB,
        HATCH_DEPLOYED_MID_TO_PREPARE_CLIMB,
        HATCH_DEPLOYED_HIGH_TO_PREPARE_CLIMB,

        //cargo systemstates
        CARGO_PREPARE_FOR_INTAKE,
        CARGO_MID,//maybe make a dedicated capture state?
        CARGO_DEPLOYED_MID,
        CARGO_HIGH,
        CARGO_DEPLOYED_HIGH,
        CARGO_LOW,
        CARGO_DEPLOYED_LOW,
        //cargo moving states
        CARGO_PREPARE_FOR_INTAKE_TO_LOW,
        CARGO_PREPARE_FOR_INTAKE_TO_MID,
        CARGO_PREPARE_FOR_INTAKE_TO_MID_SHIPMODE,
        CARGO_MID_TO_HIGH,
        CARGO_MID_TO_LOW,
        CARGO_HIGH_TO_MID,
        CARGO_LOW_TO_MID,
        CARGO_LOW_TO_PREPARE_FOR_INTAKE,
        CARGO_MID_TO_PREPARE_FOR_INTAKE,
        CARGO_HIGH_TO_PREPARE_FOR_INTAKE,
        CARGO_DEPLOYED_LOW_TO_PREPARE_FOR_INTAKE,
        CARGO_DEPLOYED_MID_TO_PREPARE_FOR_INTAKE,
        CARGO_DEPLOYED_HIGH_TO_PREPARE_FOR_INTAKE,
        CARGO_DEPLOYED_LOW_TO_PREPARE_CLIMB,
        CARGO_DEPLOYED_MID_TO_PREPARE_CLIMB,
        CARGO_DEPLOYED_HIGH_TO_PREPARE_CLIMB,
    }
    private SystemState mSystemState = SystemState.HATCH_IDLE;//hatch idle is starting config

    private boolean mStateChanged;

    private boolean wantDeploy = false;
    private boolean wantIntake = false;

    private static Superstructure mInstance = null; 
    public synchronized static Superstructure getInstance(){
        if (mInstance == null) {
        mInstance = new Superstructure();
        }
        return mInstance;
    }


    private final Loop mLoop = new Loop() {
        @Override
        public void onStart(double timestamp) {
            synchronized (Superstructure.this) {
                mStateChanged = true;
              
            }
        }
    
        @Override
        public void onLoop(double timestamp) {
            SystemState newState;
            
            synchronized (Superstructure.this) {
            switch(mSuperstructureMode){
                case HATCH:
                    handleHatchMode();
                    break;
                case CARGO: 
                    handleCargoMode();
                    break;
                case CLIMB:
                    handleClimbMode();
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
    
    //SUPER TRANSFER
    private void handleHatchMode(){
        switch(mSystemState){
            case HATCH_IDLE://starting config
                handleHatch_Idle();
                break;
            case HATCH_PREPARE_FOR_INTAKE:
                handleHatch_PrepareForIntake();
                break;
            case HATCH_LOW:
                handleHatch_Low();
                break;
            case HATCH_MID:
                handleHatch_Mid();
                break;
            case HATCH_HIGH:
                handleHatch_High();
                break;
            case HATCH_DEPLOYED_LOW:
                handleHatch_Deployed_Low();
                break;
            case HATCH_DEPLOYED_MID:
                handleHatch_Deployed_Mid();
                break;
            case HATCH_DEPLOYED_HIGH:
                handleHatch_Deployed_High();
                break;            
            case HATCH_MID_TO_HIGH:
                handleHatch_Mid_To_High();
                break;
            case HATCH_HIGH_TO_MID:
                handleHatch_High_To_Mid();
                break;
            case HATCH_MID_TO_LOW:
                handleHatch_Mid_To_Low();
                break;
            case HATCH_LOW_TO_MID:
                handleHatch_Low_To_Mid();
                break;
            case HATCH_PREPARE_FOR_INTAKE_TO_MID:
                handleHatch_PrepareForIntake_To_Mid();
                break;
            case HATCH_MID_TO_PREPARE_FOR_INTAKE:
                handleHatch_Mid_To_PrepareForIntake();
                break;
            case HATCH_HIGH_TO_PREPARE_FOR_INTAKE:
                handleHatch_High_To_PrepareForIntake();
                break;
            case HATCH_DEPLOYED_LOW_TO_PREPARE_CLIMB:
                handleHatch_DeployedLow_To_PrepareClimb();
                break;
            case HATCH_DEPLOYED_MID_TO_PREPARE_CLIMB:
                handleHatch_DeployedMid_To_PrepareClimb();
                break;
            case HATCH_DEPLOYED_HIGH_TO_PREPARE_CLIMB:
                handleHatch_DeployedHigh_To_PrepareClimb();
                break;
        }
    }
    private void handleCargoMode(){
        switch(mSystemState){
            case CARGO_PREPARE_FOR_INTAKE:
                handleCargo_PrepareForIntake();
                break;
            case CARGO_LOW:
                handleCargo_Low();
                break;
            case CARGO_MID:
                handleCargo_Mid();
                break;
            case CARGO_HIGH:
                handleCargo_High();
                break;
            case CARGO_DEPLOYED_LOW:
                handleCargo_Deployed_Low();
                break;
            case CARGO_DEPLOYED_MID:
                handleCargo_Deployed_Mid();
                break;
            case CARGO_DEPLOYED_HIGH:
                handleCargo_Deployed_High();
                break;
            case CARGO_MID_TO_HIGH:
                handleCargo_Mid_To_High();
                break;
            case CARGO_HIGH_TO_MID:
                handleCargo_High_To_Mid();
                break;
            case CARGO_MID_TO_LOW:
                handleCargo_Mid_To_Low();
                break;
            case CARGO_LOW_TO_MID:
                handleCargo_Low_To_Mid();
                break;
            case CARGO_PREPARE_FOR_INTAKE_TO_LOW:
                handleCargo_PrepareForIntake_To_Low();
                break;
            case CARGO_PREPARE_FOR_INTAKE_TO_MID:
                handleCargo_PrepareForIntake_To_Mid();
                break;
            case CARGO_PREPARE_FOR_INTAKE_TO_MID_SHIPMODE:
                handleCargo_PrepareForIntake_To_Mid_ShipMode();
                break;
            case CARGO_LOW_TO_PREPARE_FOR_INTAKE:
                handleCargo_Low_To_PrepareForIntake();
                break;
            case CARGO_MID_TO_PREPARE_FOR_INTAKE:
                handleCargo_Mid_To_PrepareForIntake();
                break;
            case CARGO_HIGH_TO_PREPARE_FOR_INTAKE:
                handleCargo_High_To_PrepareForIntake();
                break;
            case CARGO_DEPLOYED_LOW_TO_PREPARE_CLIMB:
                handleCargo_DeployedLow_To_PrepareClimb();
                break;
            case CARGO_DEPLOYED_MID_TO_PREPARE_CLIMB:
                handleCargo_DeployedMid_To_PrepareClimb();
                break;
            case CARGO_DEPLOYED_HIGH_TO_PREPARE_CLIMB:
                handleCargo_DeployedHigh_To_PrepareClimb();
                break;
        }
    }
    private void handleClimbMode(){
        switch(mSystemState){
            case PREPARING_FOR_CLIMB:
                handleClimb_PrepareClimb();
                break;
            case CLIMBING:
                handleClimb_Climb();
                break;
            case LOWER_ROBOT:
                handleClimb_LowerRobot();
            case STALL_CLIMB:
                handleClimb_StallClimb();
                break;
        }
    }

    //hatch mode handles HHHH

    public enum HatchCargoIntakeStage{//stages for the hatch to cargo prepare to intake transition.
        LIFT_ARM,
        MOVE_INTAKE_ARM,
        POSITION_ARM,
    }
    private HatchCargoIntakeStage mHatchCargoIntakeStage = HatchCargoIntakeStage.LIFT_ARM;

    public enum HatchIntakePrepareClimbStage {
        LIFT_ARM,
        LOWER_INTAKE,
        STOW_ARM,
        POSITION_INTAKE,
    }
    private HatchIntakePrepareClimbStage mHatchIntakePrepareClimbStage = HatchIntakePrepareClimbStage.LIFT_ARM;

    public enum HatchIdleToIntake{
        STOW_INTAKE,
        POSITION_ARM,
    }
    HatchIdleToIntake mHatchIdleToIntake = HatchIdleToIntake.STOW_INTAKE;
    private void handleHatch_Idle(){//this is the starting position for the robot.
        switch(mWantedState){
            case IDLE:
                break;
            case PREPARE_INTAKE:
                switch(mHatchIdleToIntake){
                    case STOW_INTAKE:
                        isInterruptible = false;
                        if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeStowed,350)){
                            mHatchIdleToIntake = HatchIdleToIntake.POSITION_ARM;
                        }
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                        mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                        mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                        break;
                    case POSITION_ARM:
                        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntakeHatch,100)){
                            isInterruptible = true;
                            wantIntake = false;
                            mWantedState = WantedState.IDLE;
                            mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
                        }
                        mClaw.openClaw();
                        // mClaw.closeBeak();
                        mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
                        mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                        break;      
                }
                
                break;
            case PREPARE_CLIMB://ONLY USE IF ARM IS BEHIND INTAKE. DEBUG STATE FOR TESTING CLIMB
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakePreparedToClimb, 100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                // mSystemState = SystemState.PREPARING_FOR_CLIMB;
                // mSuperstructureMode = SuperstructureMode.CLIMB;
                break;
        }
    }

    
    private void handleHatch_PrepareForIntake(){
        // mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
        if(wantIntake){
            mClaw.closeBeak();
        }else{
            mClaw.openBeak();
        }

        
        switch(mWantedState){
            case PREPARE_INTAKE:
                //set claw position to allow for dynamic trimming of the wrist.
                mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                // wantIntake = !wantIntake;
                // mWantedState = WantedState.IDLE;
                break;
            case WANT_CAPTURE:
                mWantedState = WantedState.GO_MID;
                break;
            case GO_MID:
                mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE_TO_MID;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.GO_MID;
                mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE_TO_MID;
                break;
            case TOGGLE_HATCH_CARGO_MODE://go to cargo PrepareForIntake
            switch(mHatchCargoIntakeStage){
                case LIFT_ARM:
                    isInterruptible = false;
                    if(mArm.getPosition() < SuperstructureConstants.kArmSafeForIntake ||
                        Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 200))
                        mHatchCargoIntakeStage = HatchCargoIntakeStage.MOVE_INTAKE_ARM;
                    mElevator.setMin();
                    mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristIntake);
                    mClaw.closeClaw();
                    break;
                case MOVE_INTAKE_ARM:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 400)){
                        mHatchCargoIntakeStage = HatchCargoIntakeStage.POSITION_ARM;
                    }
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                    mClaw.closeClaw();//maybe move closing to the mode swaps?
                    break;
                case POSITION_ARM:
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 300)){
                        isInterruptible = true;
                        mWantedState = WantedState.IDLE;
                        mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                        mSuperstructureMode = SuperstructureMode.CARGO;
                    }
                    mClaw.closeClaw();
                    mClaw.openBeak();
                    mClaw.retractPlatform();
                    mElevator.setMin();
                    mArm.setPosition(SuperstructureConstants.kArmIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristIntake);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                    break;
            }
                
                break;
            case PREPARE_CLIMB:
            switch(mHatchIntakePrepareClimbStage){
                case LIFT_ARM:
                    mClaw.retractPlatform();
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 200)){
                        mHatchIntakePrepareClimbStage = mHatchIntakePrepareClimbStage.LOWER_INTAKE;
                    }
                    mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    break;
                case LOWER_INTAKE:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLowered,200)){
                        mHatchIntakePrepareClimbStage = HatchIntakePrepareClimbStage.STOW_ARM;
                    }
                    mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                    break;
                case STOW_ARM://throw arm to the back.  Should be fine in any prepare to intake mode
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb, 300)){
                        mHatchIntakePrepareClimbStage = HatchIntakePrepareClimbStage.POSITION_INTAKE;
                    }
                    mArm.setPosition(SuperstructureConstants.kArmClimb);
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    mClaw.openClaw();
                    mClaw.openBeak();
                    break;
                case POSITION_INTAKE:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakePreparedToClimb,200)){
                        mClimbingElevator.unlockForkliftRatchet();
                        mSystemState = SystemState.PREPARING_FOR_CLIMB;
                        mSuperstructureMode = SuperstructureMode.CLIMB;
                    }
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                    break;
                }
                break;
        }
    }

    private void handleHatch_Low(){

        switch(mWantedState){
            case PREPARE_INTAKE:
                //literally in position.
                mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
                break;
            case GO_MID:
                break;
            case DEPLOY:
                mSystemState = SystemState.HATCH_DEPLOYED_LOW;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.HATCH_LOW_TO_MID;
                break;
            case BACK_STATE:
                //nothing
                break;
        }
    }

    private void handleHatch_Mid(){
        // mSystemState = SystemState.HATCH_MID;
        


        mArm.setPosition(SuperstructureConstants.kHatchHigh);
        mClaw.setPosition(SuperstructureConstants.kWristHatch);
        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.HATCH_MID_TO_PREPARE_FOR_INTAKE;
                break;
            case GO_MID:
                break;
            case DEPLOY:
                mSystemState = SystemState.HATCH_DEPLOYED_MID;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.HATCH_MID_TO_HIGH;
                break;
            case BACK_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.HATCH_MID_TO_LOW;
                break;
        }
    }

    private void handleHatch_High(){
        
        mArm.setPosition(SuperstructureConstants.kHatchHigh);
        mClaw.setPosition(SuperstructureConstants.kWristHatch);

        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.HATCH_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case GO_HIGH:
                break;
            case DEPLOY:
                mSystemState = SystemState.HATCH_DEPLOYED_HIGH;
                break;
            case FORWARD_STATE:
                //nothing
                break;
            case BACK_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.HATCH_HIGH_TO_MID;
                break;
        }
    }


    private void handleHatch_Deployed_Low(){
        mClaw.closeBeak();

        switch(mWantedState){
            case PREPARE_INTAKE:
                //already in position, just change states.
                mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
                mWantedState = WantedState.IDLE;
                break;
            case DEPLOY:
                break;
            case FORWARD_STATE:
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.HATCH_DEPLOYED_LOW_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE://go to cargo prepare for Intake
            switch(mHatchCargoIntakeStage){
                case LIFT_ARM:
                    isInterruptible = false;
                    if(mArm.getPosition() < SuperstructureConstants.kArmSafeForIntake ||
                        Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 200))
                        mHatchCargoIntakeStage = HatchCargoIntakeStage.MOVE_INTAKE_ARM;
                    mElevator.setMin();
                    mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristIntake);
                    mClaw.closeClaw();
                    break;
                case MOVE_INTAKE_ARM:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 400)){
                        mHatchCargoIntakeStage = HatchCargoIntakeStage.POSITION_ARM;
                    }
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                    mClaw.closeClaw();//maybe move closing to the mode swaps?
                    break;
                case POSITION_ARM:
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 300)){
                        isInterruptible = true;
                        mClaw.openBeak();
                        mWantedState = WantedState.IDLE;
                        mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                        mSuperstructureMode = SuperstructureMode.CARGO;
                    }
                    mClaw.closeClaw();
                    mClaw.retractPlatform();
                    mElevator.setMin();
                    mArm.setPosition(SuperstructureConstants.kArmIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristIntake);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                    break;
            }
                break;
        }
    }

    public enum HatchCargoDeployStage{//hatch mid and high use the same stages to go to cargo prepare for intake.
        LOWER_INTAKE,
        POSITION_ARM,
    }
    HatchCargoDeployStage mHatchCargoDeployStage = HatchCargoDeployStage.LOWER_INTAKE;

    private void handleHatch_Deployed_Mid(){
        mClaw.closeBeak();

        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.HATCH_MID_TO_PREPARE_FOR_INTAKE;
                break;
            case DEPLOY:
                break;
            case FORWARD_STATE:
                mSystemState = SystemState.HATCH_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.HATCH_DEPLOYED_MID_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE:
                switch(mHatchCargoDeployStage){
                    case LOWER_INTAKE:
                        isInterruptible = false;
                        if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 400)){
                            mHatchCargoDeployStage = mHatchCargoDeployStage.POSITION_ARM;
                        }
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                        mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                        mClaw.retractPlatform();
                        break;
                    case POSITION_ARM:
                        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 200)){
                            isInterruptible = true;
                            mClaw.openBeak();
                            mWantedState = WantedState.IDLE;
                            mSuperstructureMode = SuperstructureMode.CARGO;
                            mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                        }
                        mClaw.closeClaw();
                        mClaw.retractPlatform();
                        mElevator.setMin();
                        mArm.setPosition(SuperstructureConstants.kArmIntake);
                        mClaw.setPosition(SuperstructureConstants.kWristIntake);
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                        break;

                }
                break;
        }
    }

    private void handleHatch_Deployed_High(){
        mClaw.closeBeak();

        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.HATCH_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case DEPLOY:
                break;
            case FORWARD_STATE:
                mSystemState = SystemState.HATCH_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.HATCH_DEPLOYED_HIGH_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE:
                switch(mHatchCargoDeployStage){
                    case LOWER_INTAKE:
                        isInterruptible = false;
                        if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 400)){
                            mHatchCargoDeployStage = mHatchCargoDeployStage.POSITION_ARM;
                        }
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                        mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                        mElevator.setMin();
                        // mClaw.openBeak();
                        mClaw.retractPlatform();
                        break;
                    case POSITION_ARM:
                        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 200)){
                            isInterruptible = true;
                            mClaw.openBeak();
                            mWantedState = WantedState.IDLE;
                            mSuperstructureMode = SuperstructureMode.CARGO;
                            mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                        }
                        mClaw.closeClaw();
                        mClaw.retractPlatform();
                        mElevator.setMin();
                        mArm.setPosition(SuperstructureConstants.kArmIntake);
                        mClaw.setPosition(SuperstructureConstants.kWristIntake);
                        mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                        break;

                }
                break;
        }
    }

    public enum HatchLowPrepareClimb{
        LIFT_ARM,
        LOWER_INTAKE,
        STOW_ARM,
        POSITION_INTAKE,
    }
    private HatchLowPrepareClimb mHatchLowPrepareClimb = HatchLowPrepareClimb.LIFT_ARM;
    private void handleHatch_DeployedLow_To_PrepareClimb(){
        switch(mHatchLowPrepareClimb){
            case LIFT_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 100)){
                    mHatchLowPrepareClimb = HatchLowPrepareClimb.LOWER_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mHatchLowPrepareClimb = HatchLowPrepareClimb.STOW_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb,100)){
                    mHatchLowPrepareClimb = HatchLowPrepareClimb.POSITION_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }
    }
    public enum DeployedPrepareClimb{
        LOWER_INTAKE,
        STOW_ARM,
        POSITION_INTAKE,
    }
    private DeployedPrepareClimb mDeployedPrepareClimb  = DeployedPrepareClimb.LOWER_INTAKE;

    private void handleHatch_DeployedMid_To_PrepareClimb(){
        switch(mDeployedPrepareClimb){
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.STOW_ARM;
                }
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb,100)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.POSITION_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }
    }

    private void handleHatch_DeployedHigh_To_PrepareClimb(){
        switch(mDeployedPrepareClimb){
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.STOW_ARM;
                }
                mElevator.setMin();
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb,100)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.POSITION_INTAKE;
                }
                mElevator.setMin();
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }
    }

    private void handleHatch_PrepareForIntake_To_Mid(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kHatchHigh, 100)){
            mSystemState = SystemState.HATCH_MID;
        }else{
            mElevator.setMin();
            mArm.setPosition(SuperstructureConstants.kHatchHigh);
            mClaw.setPosition(SuperstructureConstants.kWristHatch);
            mClaw.openClaw();
            mClaw.openBeak();
        }
    }

    private void handleHatch_Mid_To_High(){
        if(Util.isAlmostEqual(mElevator.getPosition(), SuperstructureConstants.kElevatorMax, 500)){
            mSystemState = SystemState.HATCH_HIGH;
        }else{
            mElevator.setMax();
            mArm.setPosition(SuperstructureConstants.kHatchHigh);
            mClaw.setPosition(SuperstructureConstants.kWristHatch);
            mClaw.openClaw();
        }
    }

    private void handleHatch_Mid_To_Low(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntakeHatch, 100)){
            mSystemState = SystemState.HATCH_LOW;
        }
        mClaw.openClaw();
        mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
        mClaw.setPosition(SuperstructureConstants.kWristHatchIntake);
        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
    }

    private void handleHatch_High_To_Mid(){
        if(Util.isAlmostEqual(mElevator.getPosition(), SuperstructureConstants.kElevatorMin, 500)){
            mSystemState = SystemState.HATCH_MID;
        }else{
            mElevator.setMin();
            mArm.setPosition(SuperstructureConstants.kHatchHigh);
            mClaw.setPosition(SuperstructureConstants.kWristHatch);
            mClaw.openClaw();
        }
    }

    private void handleHatch_Low_To_Mid(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kHatchHigh, 100)){
            mSystemState = SystemState.HATCH_MID;
        }else{
            mElevator.setMin();
            mArm.setPosition(SuperstructureConstants.kHatchHigh);
            mClaw.setPosition(SuperstructureConstants.kWristHatch);
            mClaw.openClaw();
        }
    }

    private void handleHatch_High_To_PrepareForIntake(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntakeHatch, 100)){
            mWantedState = WantedState.IDLE;
            mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
        }else {
            mElevator.setMin();
            mClaw.openClaw();
            mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
            mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
            mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
        }
    }

    private void handleHatch_Mid_To_PrepareForIntake(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntakeHatch, 100)){
            mWantedState = WantedState.IDLE;
            mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
        }else {
            mElevator.setMin();
            mClaw.openClaw();
            mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
            mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
            mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
        }
    }

    //cargo handles CCCC

    public enum CargoMidStage{//cargo mid multistage
        LOWER_INTAKE,
        MOVE_ARM,
    }
    private CargoMidStage mCargoMidStage = CargoMidStage.LOWER_INTAKE;

    public enum CargoIntakeStage{//cargo intake multistage
        MOVE_INTAKE_ARM,
        POSITION_ARM,
    }
    private CargoIntakeStage mCargoIntakeStage = CargoIntakeStage.MOVE_INTAKE_ARM;

    public enum CargoHatchIntakeStage{//multi stage for the transition between cargo prepare to hatch prepare.
        LIFT_ARM,
        MOVE_INTAKE_ARM,
        POSITION_ARM,
    }
    private CargoHatchIntakeStage mCargoHatchIntakeStage = CargoHatchIntakeStage.LIFT_ARM;

    public enum CargoIntakePrepareClimbStage{
        LOWER_INTAKE,
        STOW_ARM,
        POSITION_INTAKE,
        // END,
    }
    private CargoIntakePrepareClimbStage mCargoIntakePrepareClimbStage = CargoIntakePrepareClimbStage.LOWER_INTAKE;

    private void handleCargo_PrepareForIntake(){
        if(wantIntake){
            mIntake.setOn();
            mClaw.setOn();
        }else{
            mIntake.setOff();
            mClaw.setOff();
        }

        switch(mWantedState){

            case PREPARE_INTAKE:
                // wantIntake = !wantIntake;
                // mWantedState = WantedState.IDLE;
                break;
            case  WANT_CAPTURE:
                if(shipMode){
                    mWantedState = WantedState.GO_MID;
                    mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE_TO_MID_SHIPMODE;
                }else{
                    mWantedState = WantedState.GO_MID;
                    mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE_TO_MID;
                }
                break;
            case WANT_LOW_CAPTURE:
                mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE_TO_LOW;
                break;
            case GO_MID:
                mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE_TO_MID;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.GO_MID;
                mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE_TO_MID;
                break;
            case TOGGLE_HATCH_CARGO_MODE:
                switch(mCargoHatchIntakeStage){
                case LIFT_ARM:
                    mClaw.retractPlatform();
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 200)){
                        mCargoHatchIntakeStage = CargoHatchIntakeStage.MOVE_INTAKE_ARM;
                    }
                    // mClaw.retractPlatform();
                    mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                    mClaw.setPosition(SuperstructureConstants.kWristHatchIntake);
                    break;
                case MOVE_INTAKE_ARM:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeStowed, 100)){
                        mCargoHatchIntakeStage = CargoHatchIntakeStage.POSITION_ARM;
                    }
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                    break;
                case POSITION_ARM:
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntakeHatch, 200)){
                        mWantedState = WantedState.IDLE;
                        mSystemState = SystemState.HATCH_PREPARE_FOR_INTAKE;
                        mSuperstructureMode = SuperstructureMode.HATCH;
                    }
                    mElevator.setMin();
                    mClaw.openClaw();
                    // mClaw.retractPlatform();
                    mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
                    mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                    break;
                }
                break;
            case PREPARE_CLIMB:
            wantIntake = false;
            switch(mCargoIntakePrepareClimbStage){
                case LOWER_INTAKE:
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed,100)){
                        mCargoIntakePrepareClimbStage = CargoIntakePrepareClimbStage.STOW_ARM;
                    }
                    // mClaw.openClaw();
                    mClaw.openBeak();
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                    break;
                case STOW_ARM://throw arm to the back.  Should be fine in any prepare to intake mode
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb, 300)){
                        mCargoIntakePrepareClimbStage = CargoIntakePrepareClimbStage.POSITION_INTAKE;
                    }
                    mArm.setPosition(SuperstructureConstants.kArmClimb);
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    // mClaw.openClaw();
                    mClaw.openBeak();
                    break;
                case POSITION_INTAKE:
                    mClaw.openClaw();
                    mClaw.setPosition(SuperstructureConstants.kWristClimb);
                    if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakePreparedToClimb,200)){
                        mClimbingElevator.unlockForkliftRatchet();
                        mSystemState = SystemState.PREPARING_FOR_CLIMB;
                        mSuperstructureMode = SuperstructureMode.CLIMB;
                    }
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                    break;
                }
                break;
        }
    }

    private void handleCargo_Low(){
        

        switch(mWantedState){
            case GO_MID:
                break;
            case DEPLOY:
                mSystemState = SystemState.CARGO_DEPLOYED_LOW;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.CARGO_LOW_TO_MID;
                break;
            case BACK_STATE:
                //nothing
                break;
            case PREPARE_INTAKE://go to cargo prepare to intake
                mSystemState = SystemState.CARGO_LOW_TO_PREPARE_FOR_INTAKE;
                break;
            
        }
    }    

    private void handleCargo_Mid(){
        if(shipMode){
            mArm.setPosition(SuperstructureConstants.kCargoShip);
            mClaw.setPosition(SuperstructureConstants.kWristCargoShip);
        }else{
            mArm.setPosition(SuperstructureConstants.kCargoHigh);
            mClaw.setPosition(SuperstructureConstants.kWristCargo);
        }

        switch(mWantedState){
            case GO_MID:
                break;
            case DEPLOY:
                mSystemState = SystemState.CARGO_DEPLOYED_MID;
                break;
            case FORWARD_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.CARGO_MID_TO_HIGH;
                break;
            case BACK_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.CARGO_MID_TO_LOW;
                break;
            case PREPARE_INTAKE://go to cargo prepare to intake
                mSystemState = SystemState.CARGO_MID_TO_PREPARE_FOR_INTAKE;
                break;
            
        }
    }

    private void handleCargo_High(){

        switch(mWantedState){
            case GO_HIGH:
                break;
            case DEPLOY:
                mSystemState = SystemState.CARGO_DEPLOYED_HIGH;
                break;
            case FORWARD_STATE:
                //nothing
                break;
            case BACK_STATE:
                mWantedState = WantedState.IDLE;
                mSystemState = SystemState.CARGO_HIGH_TO_MID;
                break;
            case PREPARE_INTAKE://go to cargo prepare to intake
                mSystemState = SystemState.CARGO_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            
        }
    }
    
    private void handleCargo_Deployed_Low(){
        
        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.CARGO_LOW_TO_PREPARE_FOR_INTAKE;
                break;
            case DEPLOY:
                mClaw.shoot();
                mClaw.extendPlatform();
                break;
            case FORWARD_STATE:
                mSystemState = SystemState.CARGO_LOW_TO_PREPARE_FOR_INTAKE;
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.CARGO_DEPLOYED_LOW_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE:
                isInterruptible = false;
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 100)){
                    wantIntake = true;
                    isInterruptible = true;
                    mWantedState = mWantedState.IDLE;
                    mSystemState = mSystemState.HATCH_PREPARE_FOR_INTAKE;
                    mSuperstructureMode = SuperstructureMode.HATCH;
                }
                mClaw.openClaw();
                mClaw.setOff();
                mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
                mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                break;
        }
    }

    private void handleCargo_Deployed_Mid(){
        
        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.CARGO_MID_TO_PREPARE_FOR_INTAKE;
                break;
            case DEPLOY:
                mClaw.shoot();
                mClaw.extendPlatform();
                break;
            case FORWARD_STATE:
                mSystemState = SystemState.CARGO_MID_TO_PREPARE_FOR_INTAKE;
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.CARGO_DEPLOYED_MID_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE:
                isInterruptible = false;
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 100)){
                    wantIntake = true;
                    isInterruptible = true;
                    mWantedState = mWantedState.IDLE;
                    mSystemState = mSystemState.HATCH_PREPARE_FOR_INTAKE;
                    mSuperstructureMode = SuperstructureMode.HATCH;
                }
                mClaw.openClaw();
                mClaw.setOff();
                mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                break;
        }
    }

    private void handleCargo_Deployed_High(){
        

        switch(mWantedState){
            case PREPARE_INTAKE:
                mSystemState = SystemState.CARGO_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case DEPLOY:
                mClaw.shoot();
                mClaw.extendPlatform();
                break;
            case FORWARD_STATE:
                mSystemState = SystemState.CARGO_HIGH_TO_PREPARE_FOR_INTAKE;
                break;
            case BACK_STATE:
                break;
            // case PREPARE_CLIMB:
                // mSystemState = SystemState.CARGO_DEPLOYED_HIGH_TO_PREPARE_CLIMB;
                // break;
            case TOGGLE_HATCH_CARGO_MODE:
                isInterruptible = false;
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 100)){
                    wantIntake = true;
                    isInterruptible = true;
                    mWantedState = mWantedState.IDLE;
                    mSystemState = mSystemState.HATCH_PREPARE_FOR_INTAKE;
                    mSuperstructureMode = SuperstructureMode.HATCH;
                }
                mElevator.setMin();
                mClaw.closeBeak();
                mClaw.openClaw();
                mClaw.setOff();
                mClaw.setPosition(SuperstructureConstants.kWristHatchIntake + wristTrim);
                mArm.setPosition(SuperstructureConstants.kArmIntakeHatch);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                break;
        }
    }
    
    public enum CargoLowPrepareClimb{
        LIFT_ARM,
        LOWER_INTAKE,
        STOW_ARM,
        POSITION_INTAKE,
    }
    private CargoLowPrepareClimb mCargoLowPrepareClimb = CargoLowPrepareClimb.LIFT_ARM;

    private void handleCargo_DeployedLow_To_PrepareClimb(){
        switch(mCargoLowPrepareClimb){
            case LIFT_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 100)){
                    mCargoLowPrepareClimb = CargoLowPrepareClimb.LOWER_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mCargoLowPrepareClimb = CargoLowPrepareClimb.STOW_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb,100)){
                    mCargoLowPrepareClimb = CargoLowPrepareClimb.POSITION_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }
    }

    private void handleCargo_DeployedMid_To_PrepareClimb(){
        
        switch(mDeployedPrepareClimb){
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.STOW_ARM;
                }
                mClaw.setOff();
                mClaw.openClaw();
                mClaw.openBeak();
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb, 100)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.POSITION_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }

    }

    private void handleCargo_DeployedHigh_To_PrepareClimb(){
        
        switch(mDeployedPrepareClimb){
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 300)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.STOW_ARM;
                }
                mElevator.setMin();
                mClaw.setOff();
                mClaw.openClaw();
                mClaw.openBeak();
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
            case STOW_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmClimb, 100)){
                    mDeployedPrepareClimb = DeployedPrepareClimb.POSITION_INTAKE;
                }
                mArm.setPosition(SuperstructureConstants.kArmClimb);
                mClaw.setPosition(SuperstructureConstants.kWristClimb);
                break;
            case POSITION_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb,100)){
                    mSystemState = SystemState.PREPARING_FOR_CLIMB;
                    mSuperstructureMode = SuperstructureMode.CLIMB;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                break;
        }

    }

    private void handleCargo_Mid_To_High(){
        if(Util.isAlmostEqual(mElevator.getPosition(), SuperstructureConstants.kElevatorMax, 500)){
            mSystemState = SystemState.CARGO_HIGH;
        }else{
            mElevator.setMax();
            mArm.setPosition(SuperstructureConstants.kCargoHigh);
            mClaw.setPosition(SuperstructureConstants.kWristCargo);
            mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
        }
    }

    public enum CargoMidToLowStage{
        LIFT_ARM,
        STOW_INTAKE,
        POSITION_ARM
    }

    private void handleCargo_Mid_To_Low(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmCargoLowFront, 200)){
            mSystemState = SystemState.CARGO_LOW;
        }
        mElevator.setMin();
        // mClaw.closeClaw();
        mArm.setPosition(SuperstructureConstants.kArmCargoLowFront);
        mClaw.setPosition(SuperstructureConstants.kWristCargoLowFront);
        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
        mIntake.setOff();

    }

    private void handleCargo_Low_To_Mid(){
        if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kCargoHigh, 100)){
            mSystemState = SystemState.CARGO_MID;
        }
        mElevator.setMin();
        // mClaw.closeClaw();
        mArm.setPosition(SuperstructureConstants.kCargoHigh);
        mClaw.setPosition(SuperstructureConstants.kWristCargo);
        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
        mIntake.setOff();
    }

    private void handleCargo_High_To_Mid(){
        if(Util.isAlmostEqual(mElevator.getPosition(), SuperstructureConstants.kElevatorMin, 500)){
            mSystemState = SystemState.CARGO_MID;
        }
        mElevator.setMin();
        mArm.setPosition(SuperstructureConstants.kCargoHigh);
        mClaw.setPosition(SuperstructureConstants.kWristCargo);
        mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
    }

    
    public enum CargoIntakeToLowStage{
        LIFT_ARM,
        STOW_INTAKE,
        POSITION_ARM
    }
    CargoIntakeToLowStage mCargoIntakeToLowStage = CargoIntakeToLowStage.LIFT_ARM;

    private void handleCargo_PrepareForIntake_To_Low(){
        switch(mCargoIntakeToLowStage){
            case LIFT_ARM:
                isInterruptible = false;
                mIntake.setOn();
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 100)){
                   mIntake.setOff();
                    mCargoIntakeToLowStage = CargoIntakeToLowStage.STOW_INTAKE;
                } 
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                mClaw.holdCargo();
                break;
            case STOW_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeStowed, 550)){
                    mCargoIntakeToLowStage = CargoIntakeToLowStage.POSITION_ARM;
                }
                mIntake.setOff();
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                break;
            case POSITION_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmCargoLowFront, 200)){
                    isInterruptible = true;
                    mSystemState = SystemState.CARGO_LOW;
                }
                mArm.setPosition(SuperstructureConstants.kArmCargoLowFront);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                mClaw.setPosition(SuperstructureConstants.kWristCargoLowFront);
                break;
        }

    }

    private void handleCargo_PrepareForIntake_To_Mid(){
        switch(mCargoMidStage){
            case LOWER_INTAKE:
                isInterruptible = false;
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLowered, 200)){
                    mCargoMidStage = CargoMidStage.MOVE_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                break;
            case MOVE_ARM:
                if(mArm.getPosition() > SuperstructureConstants.kArmSafeForIntake ){
                    mArm.setPosition(SuperstructureConstants.kCargoHigh);
                    mClaw.holdCargo();
                    mIntake.setOn();
                }else{
                    mArm.setPosition(SuperstructureConstants.kCargoHigh);
                    mClaw.setPosition(SuperstructureConstants.kWristCargo);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                    mIntake.setOff();
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kCargoHigh,100)){
                        isInterruptible = true;
                        mSystemState = SystemState.CARGO_MID;
                    }
                }
                break;
        } 
    }

    private void handleCargo_PrepareForIntake_To_Mid_ShipMode(){
        switch(mCargoMidStage){
            case LOWER_INTAKE:
                isInterruptible = false;
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLowered, 200)){
                    mCargoMidStage = CargoMidStage.MOVE_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                break;
            case MOVE_ARM:
                if(mArm.getPosition() > SuperstructureConstants.kArmSafeForIntake ){
                    mArm.setPosition(SuperstructureConstants.kCargoShip);
                    mClaw.holdCargo();
                    mIntake.setOn();
                }else{
                    mArm.setPosition(SuperstructureConstants.kCargoShip);
                    mClaw.setPosition(SuperstructureConstants.kWristCargoShip);
                    mIntakeArm.setPosition(SuperstructureConstants.kIntakeStowed);
                    mIntake.setOff();
                    if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kCargoShip,100)){
                        isInterruptible = true;
                        mSystemState = SystemState.CARGO_MID;
                    }
                }
                break;
        } 
    }
    
    private void handleCargo_High_To_PrepareForIntake(){
        switch(mCargoIntakeStage){
            case MOVE_INTAKE_ARM:
                isInterruptible = false;
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeDeployed, 400)){
                    mCargoIntakeStage = CargoIntakeStage.POSITION_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mElevator.setMin();
                mClaw.setOff();
                break;
            case POSITION_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 200)){
                    isInterruptible = true;
                    mWantedState = WantedState.IDLE;
                    mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                }
                mClaw.closeClaw();
                mClaw.openBeak();
                mClaw.retractPlatform();
                mClaw.setOff();
                mElevator.setMin();
                mArm.setPosition(SuperstructureConstants.kArmIntake);
                mClaw.setPosition(SuperstructureConstants.kWristIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
        }
    }

    public enum CargoLowToIntakeStage{
        LIFT_ARM,
        LOWER_INTAKE,
        POSITION_ARM,
    }
    CargoLowToIntakeStage mCargoLowToIntakeStage = CargoLowToIntakeStage.LIFT_ARM;
    
    private void handleCargo_Low_To_PrepareForIntake(){
        switch(mCargoLowToIntakeStage){
            case LIFT_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmSafeForIntake, 100)){
                    mCargoLowToIntakeStage = CargoLowToIntakeStage.LOWER_INTAKE;
                }
                mClaw.setOff();
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mClaw.setPosition(SuperstructureConstants.kWristIntake);
                break;
            case LOWER_INTAKE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLowered, 300)){
                    mCargoLowToIntakeStage = CargoLowToIntakeStage.POSITION_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mClaw.closeClaw();
                mClaw.setOff();
                break;
            case POSITION_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 200)){
                    mWantedState = WantedState.IDLE;
                    mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                }
                mClaw.closeClaw();
                mClaw.openBeak();
                mClaw.retractPlatform();
                mClaw.setOff();
                mElevator.setMin();
                mArm.setPosition(SuperstructureConstants.kArmIntake);
                mClaw.setPosition(SuperstructureConstants.kWristIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
        }
    }
    private void handleCargo_Mid_To_PrepareForIntake(){
        switch(mCargoIntakeStage){
            case MOVE_INTAKE_ARM:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLowered, 750)){
                    mCargoIntakeStage = CargoIntakeStage.POSITION_ARM;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLowered);
                mArm.setPosition(SuperstructureConstants.kArmSafeForIntake);
                mClaw.closeClaw();
                mClaw.setOff();
                break;
            case POSITION_ARM:
                if(Util.isAlmostEqual(mArm.getPosition(), SuperstructureConstants.kArmIntake, 200)){
                    mWantedState = WantedState.IDLE;
                    mSystemState = SystemState.CARGO_PREPARE_FOR_INTAKE;
                }
                mClaw.closeClaw();
                mClaw.openBeak();
                mClaw.retractPlatform();
                mClaw.setOff();
                mElevator.setMin();
                mArm.setPosition(SuperstructureConstants.kArmIntake);
                mClaw.setPosition(SuperstructureConstants.kWristIntake);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeDeployed);
                break;
        }
    }

    //Climb handles CLLL
    public enum ClimbingStage{
        LEVEL_TWO,
        LEVEL_TWO_HALF,//hab level 2 no ramp.
        LEVEL_TWO_HALF_PLUS,//hab level 2 stacked on ramp.
        PRE_LEVEL_THREE, // look for if any of our alliance members are falling off.
        LEVEL_THREE,
        MOVE_ELEVATOR_WHEEL,
        RETRACT_INTAKE_ARM,
        RETRACT_CLIMBING_ELEVATOR,
        END,
    }
    private ClimbingStage mClimbingStage = ClimbingStage.LEVEL_THREE;
    boolean firstTime = true;//to initialize climbing elevator wheels

    private void handleClimb_PrepareClimb(){
        mClaw.retractPlatform();
        mClimbingElevator.unlockForkliftRatchet();
        if(wantUnlock){
            mClimbingElevator.unlockForkliftLatch();
        }

        switch(mWantedState){
            case PREPARE_CLIMB:
                break;
            case WANT_CLIMB://3rd level climb
                mIntakeArm.selectClimbingProfile();
                mClimbingStage = ClimbingStage.PRE_LEVEL_THREE;
                mSystemState = SystemState.CLIMBING;
                break;
            case WANT_CLIMB_LEVEL_TWO://2nd level climb
                mIntakeArm.selectClimbingProfile();
                mClimbingStage = ClimbingStage.LEVEL_TWO;
                mSystemState = SystemState.CLIMBING;
                break;
        }
    }
    public double getClimbingElevatorPercentage(){//how close it is to end goal.
        return mClimbingElevator.getPosition()/SuperstructureConstants.kClimbingElevatorLevelThree;
    }
    public double getIntakeArmPercentage(){
        return (mIntakeArm.getPosition() - SuperstructureConstants.kIntakePreparedToClimb)/SuperstructureConstants.kIntakeClimb;
    }

    private void handleClimb_Climb(){
        switch(mClimbingStage){
            case LEVEL_TWO:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeLevelTwo, 100)
                    && Util.isAlmostEqual(mClimbingElevator.getPosition(), SuperstructureConstants.kClimbingElevatorLevelTwo, 100)){
                        mClimbingStage = ClimbingStage.MOVE_ELEVATOR_WHEEL;
                }
                mClimbingElevator.setWheel(0.2);
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeLevelTwo);
                mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorLevelTwo);
                break;
            case PRE_LEVEL_THREE:
                //wait for driver input before moving forward
                switch(mWantedState){
                    case FORWARD_STATE:
                        mWantedState = WantedState.IDLE;
                        mClimbingStage = ClimbingStage.LEVEL_THREE;// proceed to level three hab.
                        break;
                    case BACK_STATE:
                        mSystemState = SystemState.LOWER_ROBOT;// go back to prepare for climb
                        break;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakePreLevelThree);
                mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorLevelTwo);
                mClimbingElevator.setWheel(0.2);
                break;
            case LEVEL_THREE:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakeClimb, 100)
                    && Util.isAlmostEqual(mClimbingElevator.getPosition(), SuperstructureConstants.kClimbingElevatorLevelThree, 100)){
                        mClimbingStage = ClimbingStage.MOVE_ELEVATOR_WHEEL;
                    }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakeClimb);
                mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorLevelThree + elevatorTrim);
                mClimbingElevator.setWheel(0.2);
                switch(mWantedState){
                    case FORWARD_STATE:
                        mWantedState = WantedState.IDLE;
                        break;
                    case BACK_STATE://emergency stop climb
                        mIntakeArm.stopIntakeArm();
                        mClimbingElevator.stopElevator();
                        mWantedState = WantedState.IDLE;
                        mSystemState = SystemState.STALL_CLIMB;
                        break;
                }
                break; 
            case MOVE_ELEVATOR_WHEEL:
                if(firstTime){
                    mClimbingElevator.resetWheelEncoder();
                    // int loop = 0;
                    firstTime = false;
                }else{
                    // mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorLevelThree + elevatorTrim);
                    if(mClimbingElevator.getWheelPosition() > SuperstructureConstants.kClimbingElevatorWheelForward){
                        mClimbingElevator.setWheel(0);
                    }
                    // if(mClimbingElevator.getWheelPosition() > SuperstructureConstants.kClimbingElevatorWheelForward *(3/4)){
                    //     mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                    // }
                    mClimbingElevator.setWheel(0.3);
                }
                switch(mWantedState){
                    case FORWARD_STATE:
                        mClimbingStage = ClimbingStage.RETRACT_INTAKE_ARM;
                        break;
                    case BACK_STATE:
                        mSystemState = SystemState.LOWER_ROBOT;
                        break;
                }
                break;
            case RETRACT_INTAKE_ARM:
                if(Util.isAlmostEqual(mIntakeArm.getPosition(), SuperstructureConstants.kIntakePreparedToClimb, 100)){
                    mClimbingStage = ClimbingStage.END;
                }
                mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                break;
            case END:
                mClimbingElevator.setWheel(0);
                mSystemState = SystemState.CLIMBING;
                switch(mWantedState){
                    case FORWARD_STATE:
                        break;
                    case BACK_STATE:
                        mSystemState = SystemState.LOWER_ROBOT;
                        break;
                }
                break;
            }

            
    }

    private void handleClimb_LowerRobot(){//from pre level three back to prep climb.
        if(Util.isAlmostEqual(mClimbingElevator.getPosition(), SuperstructureConstants.kClimbingElevatorMin, 100)){
            mSystemState = SystemState.PREPARING_FOR_CLIMB;
        }
        mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorMin);
        mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
    }

    //emergency stop climb state
    private void handleClimb_StallClimb(){//saw a robot falling, had to stop climb.

        switch(mWantedState){
            case WANT_CLIMB:
                break;
            case BACK_STATE://retract elevator and arm back to the ground, hopefully safely.
                mIntakeArm.setPosition(SuperstructureConstants.kIntakePreparedToClimb);
                mClimbingElevator.setPosition(SuperstructureConstants.kClimbingElevatorMin);
                mSystemState = SystemState.PREPARING_FOR_CLIMB;
                break;

        }

    }

    private void resetStage(){
        mHatchIdleToIntake = HatchIdleToIntake.STOW_INTAKE;
        mHatchCargoIntakeStage = HatchCargoIntakeStage.LIFT_ARM;
        mHatchIntakePrepareClimbStage = HatchIntakePrepareClimbStage.LIFT_ARM;
        mHatchCargoDeployStage =  HatchCargoDeployStage.LOWER_INTAKE;
        mCargoHatchIntakeStage = CargoHatchIntakeStage.LIFT_ARM;
        mCargoIntakeStage = CargoIntakeStage.MOVE_INTAKE_ARM;
        mCargoMidStage = CargoMidStage.LOWER_INTAKE;
        mCargoIntakeToLowStage = CargoIntakeToLowStage.LIFT_ARM;
        mCargoLowToIntakeStage = CargoLowToIntakeStage.LIFT_ARM;
        mCargoIntakePrepareClimbStage = CargoIntakePrepareClimbStage.LOWER_INTAKE;

        mHatchLowPrepareClimb = HatchLowPrepareClimb.LIFT_ARM;
        mCargoLowPrepareClimb = CargoLowPrepareClimb.LIFT_ARM;
        mDeployedPrepareClimb = DeployedPrepareClimb.LOWER_INTAKE;//used for cargo/hatch deployed mid and deployed low.
    }

    private void stop() {
    }


    public void getWantIntake(){
        wantIntake = !wantIntake;
    }

    private boolean wantUnlock = false;
    public void unlockForklift(){
        wantUnlock = true;
    }

    private boolean isInterruptible = true;
    public void setWantedState(WantedState wantedState){
        wantIntake = false;
        wantDeploy = false;
        if(isInterruptible){
            resetStage();
            mWantedState = wantedState;
        }
    }

    public SuperstructureMode getMode(){
        return mSuperstructureMode;
    }
    public SystemState getSystemState(){
        return mSystemState;
    }

    private int wristTrim = 0;
    public void trimUp(){
        wristTrim = wristTrim + 10;
    }
    public void trimDown(){
        wristTrim = wristTrim - 10;
    }

    private int elevatorTrim = 0;
    public void trimElevatorUp(){
        elevatorTrim = elevatorTrim + 1;
    }
    public void trimElevatorDown(){
        elevatorTrim = elevatorTrim - 1;
    }

    private boolean shipMode = false;
    public void toggleShipMode(){
        shipMode = !shipMode;
    }

    public boolean isShipMode(){
        return shipMode;
    }

    public void updateTelemetry(){
        SmartDashboard.putString("S.SystemState", mSystemState.toString());
        SmartDashboard.putString("S.WantedState", mWantedState.toString());
        SmartDashboard.putString("S. Mode", mSuperstructureMode.toString());
        // SmartDashboard.putBoolean("ShipMode", isShipMode());
    }

    private boolean isFinished(){
        return mClaw.isFinished() && 
            mArm.isFinished() && 
            mIntakeArm.isFinished();// returns true if all subsystems have stopped moving/reached target.
    }
}
