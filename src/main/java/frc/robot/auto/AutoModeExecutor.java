/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auto;

import frc.lib.util.CrashTrackingRunnable;

/**
 * This class selects, runs, and stops (if necessary) a specified autonomous mode.
 */
public class AutoModeExecutor {
    private AutoModeBase m_auto_mode;
    private Thread m_thread = null;

    public void setAutoMode(AutoModeBase new_auto_mode) {
        m_auto_mode = new_auto_mode;
        m_thread = new Thread(new CrashTrackingRunnable() {
            @Override
            public void runCrashTracked() {
                if (m_auto_mode != null) {
                    m_auto_mode.run();
                }
            }
        });
    }

    public void start() {
        if (m_thread != null) {
            m_thread.start();
        }
    }

    public void stop() {
        if (m_auto_mode != null) {
            m_auto_mode.stop();
        }

        m_thread = null;
    }

    public AutoModeBase getAutoMode() {
        return m_auto_mode;
    }
}