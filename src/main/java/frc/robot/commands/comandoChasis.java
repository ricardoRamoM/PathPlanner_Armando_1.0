package frc.robot.commands;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.chasisSubsistema;

public class comandoChasis extends CommandBase{
    
  private final chasisSubsistema chasis;
  private boolean arcade;

    public comandoChasis(chasisSubsistema chasis, boolean arcade){
        
        this.chasis = chasis; 
        this.arcade = arcade;
       // this.arcade = arcade;
        addRequirements(chasis);
    }
    

    @Override
  public void initialize() {
    chasis.resetEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /*  double izquierda =  joystickIzq.get();
     double derecha =  joystickDer.get();
    */
     chasis.manejar(arcade);

}
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  public void cambiarATanke(boolean arcadeD){
    arcade = false;
  }

  public void cambiarAArcade(){
    arcade = true;
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

