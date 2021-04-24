/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.shootercommands.ShootCommand;
import frc.robot.commands.shootercommands.StopShooting;
import frc.robot.subsystems.Shooter;

public class ShootFromAutoLine extends SequentialCommandGroup {
  /**
   * Creates a new ShootFromAutoLine.
   */
  public ShootFromAutoLine() {
    super(new ShootCommand(), new WaitCommand(1), new InstantCommand(RobotContainer.intake::deploy, RobotContainer.intake));
    // Use addRequirements() here to declare subsystem dependencies.
  }
}
//, new StopShooting() new WaitCommand(7)