/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Shooter;

public class ShootLeaveLine extends SequentialCommandGroup {
  /**
   * Creates a new ShootLeaveLine.
   */
  public ShootLeaveLine(DriveTrain driveTrain, Shooter shooter) {
    super(new ShootFromAutoLine(shooter),new AutoPlanCMovesOffLine(driveTrain));
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
