// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.blinkincommands.SetHotPink;
import frc.robot.commands.blinkincommands.SetWhite;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousMovingPractice extends SequentialCommandGroup {
  /** Creates a new AutonomousMovingPracitce. */
  public AutonomousMovingPractice() {
    super( 
    new DriveDistance(-15),
    new SetHotPink(),
    new WaitCommand(1),
    new DriveDistance(15),
    new SetWhite());
    
    
  }
}
