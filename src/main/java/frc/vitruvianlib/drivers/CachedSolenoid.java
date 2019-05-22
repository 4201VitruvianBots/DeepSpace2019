package frc.vitruvianlib.drivers;

import edu.wpi.first.hal.SolenoidJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SensorUtil;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * This class is a thin wrapper around the DoubleSolenoid class that reduces CAN bus / CPU overhead by skipping 
 * duplicate get/set commands.
 * This was inspired from team 254's LazyTalonSRX implementation.
 */
public class CachedSolenoid extends Solenoid {
	protected boolean mLastState = false;

	public CachedSolenoid(int channel) {
		super(SensorUtil.getDefaultSolenoidModule(), channel);
		// TODO Auto-generated constructor stub
		mLastState = super.get();
	}

	public CachedSolenoid(int moduleNumber, int channel) {
		super(moduleNumber, channel);
		// TODO Auto-generated constructor stub
		mLastState = super.get();
	}

	@Override
	public void set(boolean on) {
		if(on != mLastState) {
			super.set(on);
			mLastState = super.get();
		}
	}

	@Override
	public boolean get() {
		return mLastState;
	}
}
