/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.auto.AutoModeBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;

public class AutoModeSelector {

    enum DesiredMode {
        DO_NOTHING,
        CROSS_HAB_LINE,
        ADVANCED, // This uses 4 additional sendable choosers to pick one for each field state combo
    };

    private DesiredMode mCachedDesiredMode = null;
    
    private SendableChooser<DesiredMode> mModeChooser;

    // private Optional<AutoModeCreator> mCreator = Optional.empty();


    public AutoModeSelector() {
        mModeChooser = new SendableChooser<>();
        mModeChooser.addDefault("Cross Hab Line", DesiredMode.CROSS_HAB_LINE);
        mModeChooser.addDefault("Do Nothing", DesiredMode.DO_NOTHING);
        SmartDashboard.putData("Auto mode", mModeChooser);

    }

    public void updateModeCreator() {
        DesiredMode desiredMode = mModeChooser.getSelected();
        if (mCachedDesiredMode != desiredMode) {
            System.out.println("Auto selection changed, updating creator: desiredMode->" + desiredMode.name());
            
        }
        mCachedDesiredMode = desiredMode;
    }

    public void reset() {
        mCachedDesiredMode = null;
    }

    public void outputToSmartDashboard() {
        SmartDashboard.putString("AutoModeSelected", mCachedDesiredMode.name());
    }

}