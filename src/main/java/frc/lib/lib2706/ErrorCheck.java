package frc.lib.lib2706;

import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.DriverStation;

public class ErrorCheck {
    private static final int MAXIMUM_RETRIES = 5;

    public static boolean errREV(REVLibError error) {
        if (error == REVLibError.kOk) {
            return true;
        }
        DriverStation.reportError("REV DEVICE Error" + error.toString(), true);
        return false;
    }

    public static boolean errCTRE(ErrorCode error) {
        if (error == ErrorCode.OK) {
            return true;
        }

        DriverStation.reportError("CTRE Device Error: " + error.toString(), true);
        return false;
    }

    /**
     * Configure a SparkMax setting multiple times until it succeeds.
     * 
     * @param config The Supplier to call to configure which returns a REVLibError.
     * @return true for success, false for failure.
     */
    public static boolean errSpark(Supplier<REVLibError> config) {
        REVLibError err = REVLibError.kOk;
        for (int i = 0; i < MAXIMUM_RETRIES; i++) {
            err = config.get();
            if (err == REVLibError.kOk) {
                return true;
            }
        }

        DriverStation.reportError(String.format(
            "CANSparkMax failed to configure setting. Error: %s \nSee stack trace.", 
            err.toString()), 
            true);
            
        return false;
    }

}
