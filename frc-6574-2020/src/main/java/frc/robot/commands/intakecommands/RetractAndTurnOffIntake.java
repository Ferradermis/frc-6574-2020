// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RetractAndTurnOffIntake extends ParallelCommandGroup {
  /** Creates a new RetractAndTurnOffIntake. */
  public RetractAndTurnOffIntake() {
    super(
      new IntakeRetract(),
      new TurnIntakeOff()
    );
  }
}
