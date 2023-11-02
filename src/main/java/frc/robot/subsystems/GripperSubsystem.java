package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class GripperSubsystem  extends SubsystemBase{
    private DoubleSolenoid doubleSolenoid1;
    private DoubleSolenoid doubleSolenoid2;

    public Timer solenoid1Timer = new Timer();
    public Timer solenoid2Timer = new Timer();

    private static GripperSubsystem instance;
    public static GripperSubsystem getInstance(){
        if(instance == null){
            instance = new GripperSubsystem();
        }
        return instance;
    }

    // Create ArmPneumaticsSubsystem
    private GripperSubsystem(){
        doubleSolenoid1 = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Config.INTAKE1_PNEUMATIC_FORWARD_CHANNEL, Config.INTAKE1_PNEUMATIC_REVERSE_CHANNEL);
        doubleSolenoid2 = new DoubleSolenoid(Config.CTRE_PCM_CAN_ID, PneumaticsModuleType.CTREPCM, Config.INTAKE2_PNEUMATIC_FORWARD_CHANNEL, Config.INTAKE2_PNEUMATIC_REVERSE_CHANNEL);
    }

    // Low Pressure
    public void lowPressure(boolean turnOffAfterHalfSecond) {
        doubleSolenoid1.set(Value.kForward);
        doubleSolenoid2.set(Value.kReverse);
        if (turnOffAfterHalfSecond) {
            solenoid1Timer.restart();
            solenoid2Timer.restart();
        }
    }

    // High Pressure
    public void highPressure(boolean turnOffAfterHalfSecond) {
        doubleSolenoid1.set(Value.kForward);
        doubleSolenoid2.set(Value.kForward);
        if (turnOffAfterHalfSecond) {
            solenoid1Timer.restart();
            solenoid2Timer.restart();
        }
    }

    // No Pressure
    public void noPressure(boolean turnOffAfterHalfSecond) {
        doubleSolenoid1.set(Value.kReverse);
        doubleSolenoid2.set(Value.kReverse);
        if (turnOffAfterHalfSecond) {
            solenoid1Timer.restart();
            solenoid2Timer.restart();
        }
    }

    // power off both solenoids electronically
    public void stop() {
        doubleSolenoid1.set(Value.kOff);
        doubleSolenoid2.set(Value.kOff);
    }

    // low pressure command
    public Command lowPressureCommand() {
        return this.runOnce(() -> lowPressure(false));
    }

    // high pressure command
    public Command highPressureCommand() {
        return this.runOnce(() -> highPressure(false));
    }

    // no pressure command
    public Command noPressureCommand() {
        return this.runOnce(() -> noPressure(false));
    }

    // power off both solenoids electronically command
    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // check if solenoid 1 timer has elapsed
        if (solenoid1Timer.hasElapsed(0.5)) {
            doubleSolenoid1.set(Value.kOff);
            solenoid1Timer.stop();
            solenoid1Timer.reset();
        }

        // check if solenoid 2 timer has elapsed
        if (solenoid2Timer.hasElapsed(0.5)) {
            doubleSolenoid2.set(Value.kOff);
            solenoid2Timer.stop();
            solenoid2Timer.reset();
        }
    }
}
