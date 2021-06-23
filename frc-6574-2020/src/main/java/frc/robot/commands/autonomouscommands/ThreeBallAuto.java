/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class ThreeBallAuto extends SequentialCommandGroup {
  /**
   * Creates a new ShootLeaveLine.
   */
  public ThreeBallAuto() {
    super(new ShootFromAutoLine(), new DriveDistance(3));
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
