/*package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class subintake extends SubsystemBase {


    private final DoubleSolenoid PistonIntake = 
        new DoubleSolenoid(PneumaticsModuleType.REVPH, 
        // si talvez marca error, cambia al tipo de solennoide a CTREPCM
        1, 2);

    private final CANSparkMax MotorIntake = new CANSparkMax(0, MotorType.kBrushless);

    public subintake(){
    }

    public void setIntake(){
        PistonIntake.set(DoubleSolenoid.Value.kReverse);
    }

    //To change pistons 
    public void Toogle(){
    PistonIntake.toggle();
    }
    
    public void motorForward(){
        MotorIntake.set(5);
    }

    public void destrabar(){
      MotorIntake.set(-0.5);
    }
    
    public void stop(){
    PistonIntake.set(DoubleSolenoid.Value.kReverse);
    MotorIntake.set(0);
    } 

}*/
