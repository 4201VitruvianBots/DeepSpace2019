package frc.vitruvianlib.drivers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

/**
 * This class is a thin wrapper around the VictorSPX class that reduces CAN bus / CPU overhead by skipping duplicate 
 * set commands. (By default the Talon flushes the Tx buffer on every set call).
 * This was inspired from team 254's LazyTalonSRX implementation.
 */
public class CachedVictorSPX extends VictorSPX {
    protected double mLastSet = Double.NaN;
    protected ControlMode mLastControlMode = null;

    public CachedVictorSPX(int deviceNumber) {
        super(deviceNumber);
        super.configFactoryDefault();
    }

    public double getLastSet() {
        return mLastSet;
    }

    @Override
    public void set(ControlMode mode, double value) {
        if (value != mLastSet || mode != mLastControlMode) {
            mLastSet = value;
            mLastControlMode = mode;
            super.set(mode, value);
        }
    }
}
