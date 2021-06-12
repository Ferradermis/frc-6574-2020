// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.blinkincommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autonomouscommands.DriveDistance;

/** Add your docs here. */
public class Rainbow extends SequentialCommandGroup{
    static double delay = .3;

    public Rainbow() {

        // Rainbow!!!!!!!!!!!
        super(
            new DriveDistance(1), 
            new SetHotPink(), 
            new WaitCommand(delay),
            new DriveDistance(-1),
            new SetRed(),
            new WaitCommand(delay),
            new DriveDistance(2),
            new SetOrange(),
            new WaitCommand(delay),
            new DriveDistance(-2),
            new SetYellow(),
            new WaitCommand(delay),
            new DriveDistance(3),
            new SetLawnGreen(),
            new WaitCommand(delay),
            new DriveDistance(-3),
            new SetSkyBlue(),
            new WaitCommand(delay),
            new DriveDistance(4),
            new SetBlue(),
            new WaitCommand(delay),
            new DriveDistance(-4),
            new SetViolet(),
            new WaitCommand(delay),
            new DriveDistance(5),
            new SetWhite(),
            new WaitCommand(delay),
            new DriveDistance(-5),
            new SetBlack(),
            new WaitCommand(delay),
            new DriveDistance(5),
            new SetBlack(),
            new WaitCommand(delay),
            new DriveDistance(-5),
            new SetWhite(),
            new WaitCommand(delay),
            new DriveDistance(4),
            new SetViolet(),
            new WaitCommand(delay),
            new DriveDistance(-4),
            new SetBlue(),
            new WaitCommand(delay),
            new DriveDistance(3),
            new SetSkyBlue(),
            new WaitCommand(delay),
            new DriveDistance(-3),
            new SetLawnGreen(),
            new WaitCommand(delay),
            new DriveDistance(2),
            new SetYellow(),
            new WaitCommand(delay),
            new DriveDistance(-2),
            new SetOrange(),
            new WaitCommand(delay),
            new DriveDistance(1),
            new SetRed(),
            new WaitCommand(delay),
            new DriveDistance(-1),
            new SetHotPink());
    
    }
}
