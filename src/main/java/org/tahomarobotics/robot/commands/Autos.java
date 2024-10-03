// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.tahomarobotics.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.tahomarobotics.robot.subsystems.ExampleSubsystem;



public final class Autos
{
    /** Example static factory for an autonomous command. */
    public static Command exampleAuto(ExampleSubsystem subsystem)
    {
        return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
    }
    
    
    private Autos()
    {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
