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
import frc.robot.subsystems.Shooter;


public class StopShooting extends SequentialCommandGroup {
  /**
   * Creates a new StopShooting.
   */
  public StopShooting(Shooter shooter) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    super(
      new ParallelCommandGroup(
        new InstantCommand(RobotContainer.turret::stopAiming, RobotContainer.turret),
        new InstantCommand(()->shooter.lowerHoodForTrench()),
        new InstantCommand(shooter::stopShooter, shooter)
       )
      //new WaitCommand(.4),
      //new InstantCommand(()->shooter.retractHoodforShortDistance())
    );
  }
}
