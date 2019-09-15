package frc.vitruvianlib.drivers;

import edu.wpi.first.hal.SolenoidJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.SensorUtil;

/**
 * This class is a thin wrapper around the DoubleSolenoid class that reduces CAN bus / CPU overhead by skipping 
 * duplicate get/set commands.
 * This was inspired from team 254's LazyTalonSRX implementation.
 */
public class CachedDoubleSolenoid extends DoubleSolenoid {
	protected Value mLastValue = null;
	public CachedDoubleSolenoid(int forwardChannel, int reverseChannel) {
		super(SensorUtil.getDefaultSolenoidModule(), forwardChannel, reverseChannel);
		mLastValue = super.get();
	}

	public CachedDoubleSolenoid(int moduleNumber, int forwardChannel, int reverseChannel) {
		super(moduleNumber, forwardChannel, reverseChannel);
		mLastValue = super.get();
	}

	@Override
	public Value get() {
		return mLastValue;
	}

	@Override
	public void set(final Value value) {
		if (mLastValue != value) {
			super.set(value);
			mLastValue = super.get();
		}
	}
}
