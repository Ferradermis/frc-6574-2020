// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomouscommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intakecommands.DeployAndTurnOnIntake;
import frc.robot.commands.spindexercommands.SpindexerShake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class StraightLineSixBallAuto extends SequentialCommandGroup {
  /** Creates a new StraightLineSixBallAuto. */
  public StraightLineSixBallAuto() {
    super(new ShootFromAutoLine(), 
          new DeployAndTurnOnIntake(),
          new DriveDistance(15), 
          new WaitCommand(.1), 
          new ParallelCommandGroup(new DriveDistance(-14), new SpindexerShake()),
          new ShootFromAutoLine());
    }
}
