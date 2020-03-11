/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;



public class AutoTest extends SequentialCommandGroup {
  
  public AutoTest() {
    addCommands(
                new InstantCommand(()->RobotContainer.shooter.defaultShooterOn()),
                new InstantCommand(()->HelperMethods.allAutoStart()),
                new DriveAlongAngle(2, 0),
                new TurnToHeading(-45),
               // THEN TRY
               // new AutoShootCommand(),
                // new DriveAlongAngle(2, -45),
                // new TurnToHeading(0),
                // new DriveAlongAngle(5, 0, .75),
                 new InstantCommand(()->HelperMethods.allAutoEnd())
              );
  }
}

  // Called when the command is initially scheduled.
//  @Override
//  public void initialize() {
//    HelperMethods.allAutoStart();
        
  // driveTrain.driveAlongAngle(2, 0);
  // RobotContainer.shoot.schedule();
  // Timer.delay(10);
  // driveTrain.driveAlongAngle(2, 0);
    
//    HelperMethods.allAutoEnd();
//  }
//}