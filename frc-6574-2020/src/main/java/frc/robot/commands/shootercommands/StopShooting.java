/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;


public class StopShooting extends SequentialCommandGroup {
  /**
   * Creates a new StopShooting.
   */
  public StopShooting() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new InstantCommand(RobotContainer.turret::stopAiming, RobotContainer.turret),
        new InstantCommand(RobotContainer.shooter::stopShooter, RobotContainer.shooter)
       ),
      //new InstantCommand(()->RobotContainer.shooter.retractHoodforShortDistance()), 
      //new WaitCommand(.1),
      new InstantCommand(()->RobotContainer.shooter.lowerHoodForTrench())
    );
  }
}
