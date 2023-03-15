package frc.robot.subsystems;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TriangleSubsystemConstants;

public class TriangleSubsystem extends SubsystemBase{
    private TalonSRX right;
    private TalonSRX left;

    public TriangleSubsystem(){
        right = new TalonSRX(TriangleSubsystemConstants.rightMotor);
        right.configFactoryDefault();
        right.setNeutralMode(NeutralMode.Brake);
        right.setSelectedSensorPosition(0);
        // right.config_kF(0, getEncoderPos());
        // right.config_kP(0, getEncoderPos())
        left = new TalonSRX(TriangleSubsystemConstants.leftMotor);
        left.configFactoryDefault();
        left.setNeutralMode(NeutralMode.Brake);
        left.setSelectedSensorPosition(0);
        // right.configReverseSoftLimitThreshold(6500);
        // right.configReverseSoftLimitEn>able(true);
        //yessir
    }
    public void set(double x){
        right.set(TalonSRXControlMode.PercentOutput, 0.8 * x);
        left.set(TalonSRXControlMode.PercentOutput, 0.8 * x);
    }

    public void setAngle()
    {
        //System.out.print("test");
        // SmartDashboard.putNumber("angle", right.getSelectedSensorPosition(0));
        // System.out.println("please");
        // right.set(ControlMode.PercentOutput, 0.2);
        right.set(TalonSRXControlMode.Position, 5888);
        // right.setSelectedSensorPosition(3200); // FIXME change to desire encoder value
    }

    public double getEncoderPos() {
        return right.getSelectedSensorPosition();
    }
   //public void setLevel()
   

    public enum Level {
        
    }
}
