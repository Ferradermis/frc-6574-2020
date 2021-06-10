package frc.robot.commands.shootercommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class JustShootAuto extends SequentialCommandGroup {
  public JustShootAuto() {
    super (
      new WaitCommand(.25),
      new FeedAndFire(),
      new WaitCommand(1)
    );
  }
}