package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawSubsystemConstants;

//all your methods will go here, and command just helps executes the methods
public class ClawSubsystem extends SubsystemBase{
    private DoubleSolenoid right;
    // private DoubleSolenoid left;
    public ClawSubsystem(){
        
        right = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ClawSubsystemConstants.rightPnuematicInput, ClawSubsystemConstants.rightPnuematicOutput);
            }
    public void extend(){
        right.set(Value.kForward);
    }
    public void retract(){
        right.set(Value.kReverse);
    }
}
