package frc.robot;
import java.io.IOException;
import java.nio.file.Path;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.constantesRamsetController;
import frc.robot.subsystems.chasisSubsistema;

public class LectorTrajectoria {
    /*static chasisSubsistema chasis = chasisSubsistema.getInstance();

    public static Command cargarTrajectoriaARamsetePP(PathPlannerTrajectory trajectory, Boolean resetearOdometria){
      return new SequentialCommandGroup(
       new InstantCommand(() -> {
           if(resetearOdometria){
               chasis.resetOdometria(trajectory.getInitialPose());
           }
       }
      ),

       new PPRamseteCommand(trajectory, 
       chasis::getPose, 
       new RamseteController(constantesRamsetController.kRamseteB, constantesRamsetController.kRamseteZeta), 
       new SimpleMotorFeedforward(constantesRamsetController.ksVolts, constantesRamsetController.kvVoltsPerMeter, constantesRamsetController.kaVoltsSquaredPerMeter), 
       constantesRamsetController.kinematicasDrive, 
       chasis::getWheelSpeeds, 
       new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
       new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
       chasis::chasisVoltje, 
       chasis)
     );
      }

    //CARGAR NORMAL TRAJECTORIA*******************************************
    public static Command cargarTrajectoriaARamsetePW(String trajectory, Boolean resetearOdometria){
        Trajectory trajectoria;
        
        try {
          Path trajectoriPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory);
          trajectoria = TrajectoryUtil.fromPathweaverJson(trajectoriPath);
        }catch (IOException excepcion){
          DriverStation.reportError("No se pudo abrir la trajectoria" + trajectory, excepcion.getStackTrace());
          System.out.println("no se pudo leer el archivo " + trajectory);
          return new InstantCommand();
        }
    
        RamseteCommand ramseteCommand = new RamseteCommand(trajectoria, 
        chasis::getPose, 
        new RamseteController(constantesRamsetController.kRamseteB, constantesRamsetController.kRamseteZeta), 
        new SimpleMotorFeedforward(constantesRamsetController.ksVolts, constantesRamsetController.kvVoltsPerMeter, constantesRamsetController.kaVoltsSquaredPerMeter), 
        constantesRamsetController.kinematicasDrive, 
        chasis::getWheelSpeeds, 
        new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
        new PIDController(constantesRamsetController.kpDriveVelocity, 0, 0), 
        chasis::chasisVoltje, 
        chasis);
      
         if(resetearOdometria){
          return new SequentialCommandGroup(new InstantCommand(()->chasis.resetOdometria(trajectoria.getInitialPose())), ramseteCommand);
        } else {
          return ramseteCommand;
      }

    

*/
}