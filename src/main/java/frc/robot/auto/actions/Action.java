/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto.actions;

/**
 * An interface that describes an iterative action. It is run by an autonomous action, called by the
 * method runAction in AutoModeBase (or more commonly in autonomous modes that extend AutoModeBase)
 *
 * @see com.team254.frc2018.auto.AutoModeBase#runAction
 */
public interface Action {
    /**
     * Returns whether or not the code has finished execution. When implementing this interface, this method is used by
     * the runAction method every cycle to know when to stop running the action
     *
     * @return boolean
     */
    boolean isFinished();

    /**
     * Called by runAction in AutoModeBase iteratively until isFinished returns true. Iterative logic lives in this
     * method
     */
    void update();

    /**
     * Run code once when the action finishes, usually for clean up
     */
    void done();

    /**
     * Run code once when the action is started, for set up
     */
    void start();
}
