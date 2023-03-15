package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommandExtend extends CommandBase{
    private final ArmSubsystem m_ArmSubsystem;
    private final CommandPS4Controller m_controller;
    private boolean flag;

    public ArmCommandExtend(ArmSubsystem claw, CommandPS4Controller controller){
        flag = false;
        m_ArmSubsystem = claw;
        m_controller = controller;
        addRequirements(m_ArmSubsystem);
    }

    @Override
    public void execute() {
        if(m_controller.getRawAxis(2) >= 0.25){
            flag = true;
            m_ArmSubsystem.set(0.6);
        }
        else{
            if(!flag){
                m_ArmSubsystem.set(0);
            }
            flag = false;
        }
        if(m_controller.getRawAxis(3) >= 0.25){
            flag = true;
            m_ArmSubsystem.set(-0.6);
        }
        else{
            if(!flag){
                m_ArmSubsystem.set(0);
            }
            flag = false;
        }
        // m_ArmSubsystem.set(m_controller.getRawAxis(4));
    }
}







