// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.spindexercommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpindexerShake extends SequentialCommandGroup {
  final static double CCW = 1;
  final static double CW = .5;
  /** Creates a new SpindexerShake. */
  public SpindexerShake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(
      new SpindexerCCW(CCW),
      new SpindexerCW(CW),
      new SpindexerCCW(CCW),
      new SpindexerCW(CW),
      new SpindexerCCW(CCW),
      new SpindexerCW(CW),
      new SpindexerCCW(CCW),
      new SpindexerCW(CW),
      new StopSpindexer()
    );

  }
}
