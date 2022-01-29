/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double ksVolts = 0.5849;
    public static final double kvVoltSecondsPerMeter = 2.628;
    public static final double kaVoltSecondsSquaredPerMeter = 0.34565;

    public static final double kPDriveVel = 3.5201;//3.561;

    public static final double kTrackWidthMeters = 0.635;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 1.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Explanation:
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/ramsete.html
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
