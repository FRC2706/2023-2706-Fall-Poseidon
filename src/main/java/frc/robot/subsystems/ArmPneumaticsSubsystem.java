package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ArmPneumaticsSubsystem  extends SubsystemBase{
    private DoubleSolenoid topBrakeSolenoid;
    private DoubleSolenoid bottomBrakeSolenoid;

    private static ArmPneumaticsSubsystem instance;
    public static ArmPneumaticsSubsystem getInstance(){
        if(instance == null){
            instance = new ArmPneumaticsSubsystem();
        }
        return instance;
    }

    // Create ArmPneumaticsSubsystem
    private ArmPneumaticsSubsystem(){
        topBrakeSolenoid = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Config.ARMHIGH_PNEUMATIC_FORWARD_CHANNEL, Config.ARMHIGH_PNEUMATIC_REVERSE_CHANNEL);
        bottomBrakeSolenoid = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Config.ARMLOW_PNEUMATIC_FORWARD_CHANNEL, Config.ARMLOW_PNEUMATIC_REVERSE_CHANNEL);
    }

    // control top brake
    public void controlTopBrake(boolean brakeOn, boolean turnOffAfterHalfSecond) {
        if (brakeOn) {
            topBrakeSolenoid.set(Value.kForward);
        } else {
            topBrakeSolenoid.set(Value.kReverse);
        }
        if (turnOffAfterHalfSecond) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            topBrakeSolenoid.set(Value.kOff);
        }
    }

    // control bottom brake
    public void controlBottomBrake(boolean brakeOn, boolean turnOffAfterHalfSecond) {
        if (brakeOn) {
            bottomBrakeSolenoid.set(Value.kForward);
        } else {
            bottomBrakeSolenoid.set(Value.kReverse);
        }
        if (turnOffAfterHalfSecond) {
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
            bottomBrakeSolenoid.set(Value.kOff);
        }
    }
}
