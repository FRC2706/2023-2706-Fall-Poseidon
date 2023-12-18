package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;

public class ArmPneumaticsSubsystem  extends SubsystemBase{
    private DoubleSolenoid topBrakeSolenoid;
    private DoubleSolenoid bottomBrakeSolenoid;

    public Timer topBrakeTimer = new Timer();
    public Timer bottomBrakeTimer = new Timer();

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
            topBrakeTimer.restart();
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
            bottomBrakeTimer.restart();
        }
    }

    /**
     * Get a command to control the bottom brake.
     * 
     * @param brakeOn True to turn the brake on, false to release the brake.
     * @return The command to control the brake.
     */
    public Command getBottomBrakeCommand(boolean brakeOn) {
        return runOnce(() -> controlBottomBrake(brakeOn, true));
    }

    /**
     * Get a command to control the top brake.
     * 
     * @param brakeOn True to turn the brake on, false to release the brake.
     * @return The command to control the brake.
     */
    public Command getTopBrakeCommand(boolean brakeOn) {
        return runOnce(() -> controlTopBrake(brakeOn, true));
    }

    public Command getToggleBottomCommand() {
        return startEnd(() -> controlBottomBrake(false, true),
                        () -> controlBottomBrake(true, true));
    }

    public Command getToggleTopCommand() {
        return startEnd(() -> controlTopBrake(false, true),
                        () -> controlTopBrake(true, true));
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // check if top brake timer has elapsed
        if (topBrakeTimer.hasElapsed(0.5)) {
            topBrakeSolenoid.set(Value.kOff);
            topBrakeTimer.stop();
            topBrakeTimer.reset();
        }

        // check if bottom brake timer has elapsed
        if (bottomBrakeTimer.hasElapsed(0.5)) {
            bottomBrakeSolenoid.set(Value.kOff);
            bottomBrakeTimer.stop();
            bottomBrakeTimer.reset();
        }
    }
}
