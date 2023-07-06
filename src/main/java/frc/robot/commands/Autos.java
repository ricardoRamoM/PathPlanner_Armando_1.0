// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LectorTrajectoria;
import frc.robot.RobotContainer;

public final class Autos extends CommandBase{
  /** Example static factory for an autonomous command. */
 //private static PathPlannerTrajectory avanzarDerecho = PathPlanner.loadPath("PRUEBAA", new PathConstraints(1.5, 3));



    public static Command avanzarDerecho(){
      
      PathPlannerTrajectory avanzarDerechoPath =PathPlanner.loadPath("New Path", new PathConstraints(.5, 1));

      return Commands.sequence(RobotContainer.cargarTrajectoriaARamsetePP(avanzarDerechoPath, true));
    }

  
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
