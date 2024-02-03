/*package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subintake;  //Importar los subsitemas requeridos para este comando

public class comintake extends Command {

  private final subintake subintake; //to take acces of the subintake

  int prevToggleMotor;
  int ToggleMotor;

  public comintake(subintake subintake, int prevToggleMotor, int ToggleMotor){
      this.prevToggleMotor = prevToggleMotor;
      this.ToggleMotor = ToggleMotor;
      this.subintake = subintake;

      addRequirements(subintake);

    }
    @Override
    public void initialize() {
      subintake.setIntake();
    }

    @Override
    public void execute(){

      int prevToggleMotor = 0;

      //funcion para boton principal

      if (prevToggleMotor == ToggleMotor){
        if (ToggleMotor==1){
          ToggleMotor=0;
          subintake.stop();
          
        } else{
          ToggleMotor=1;
          subintake.motorForward();
          subintake.Toogle();
        }
        
      }else {
        ToggleMotor = prevToggleMotor; // debe cambiar a 0
        subintake.stop();
      }
    }

    public void destrabar(){
      subintake.destrabar();
    }

  @Override
    public boolean isFinished() {
    subintake.stop();
    return false;
    }

  }*/