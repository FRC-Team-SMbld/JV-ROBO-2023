package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmSubsystemConstants;

public class ArmSubsystem extends SubsystemBase{
    private VictorSPX motor;

    //yo thanks for the help, can you check the code for the encoder, idk the ticks per rotation, you can ask for the uses of the encoder
    // thanks again, all love ;)

    //im going to watch my friends at swim lolz the encoder code looks fine (where did u get ticks per rotation from) also literally just test to see if it goes the right way when you run it, if it goes fucking haywire the encoder
    //is in the wrong way and if it just goes the wrong way put a negative somewhere 
    public ArmSubsystem(){
        motor = new VictorSPX(ArmSubsystemConstants.rightMotor);
    }

    public void set(double x){
        motor.set(VictorSPXControlMode.PercentOutput, x);
    }
}
