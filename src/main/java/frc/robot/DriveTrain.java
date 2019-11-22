package frc.robot;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.CANSparkMaxFrames;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Handles actions related to robot motion. Includes motor controller, encoder,
 * and gyro instances. Created January 2018
 * 
 * @author Brewer FIRST Robotics Team 4564
 * @author Evan McCoy
 * @author Brent Roberts
 */
public class DriveTrain extends DifferentialDrive {
	private static DriveTrain instance;

	public int runcount = 0;
	public static double DRIVEACCEL = 0.04, DRIVEMIN = 0.4;

	public static final double TURNACCEL = 0.06;
	public static final double TANKACCEL = 0.055;
	public static final double TANKMIN = 0.40;
	public static final double TURNMAX = 0.75;
	public static final double TANKMAX = 0.4;


	public static final double DEADZONE = 0.06;

	//private static final double DISTANCE_PER_PULSE_L = 0.0098195208, DISTANCE_PER_PULSE_R = 0.0098293515;
	private static final CANSparkMax frontL = new CANSparkMax(Constants.DRIVE_FL, CANSparkMax.MotorType.kBrushless), frontR = new CANSparkMax(Constants.DRIVE_FR, CANSparkMax.MotorType.kBrushless),
			backL = new CANSparkMax(Constants.DRIVE_BL, CANSparkMax.MotorType.kBrushless), backR = new CANSparkMax(Constants.DRIVE_BR, CANSparkMax.MotorType.kBrushless);
	private static final SpeedControllerGroup left = new SpeedControllerGroup(frontL, backL);
	private static final SpeedControllerGroup right = new SpeedControllerGroup(frontR, backR);

	private CANPIDController frontRPID;

	private CANEncoder encoderL, encoderR;
	private static final double highGearCPI = 0.06927503103, lowGearCPI = 0.05320322383;

	//private PID pidL, pidR;
	//private Heading heading;
    private Solenoid gearboxSolenoid;
	private double driveSpeed = 0, turnSpeed = 0;
	private double tankLeft = 0, tankRight = 0;
	private boolean inHighGear = true;
	
	private double P, I, D, FF, minOutput, maxOutput;

	/**
	 * Creates an instance of DriveTrain. Motor controller and encoder channels are
	 * determined in Constants.
	 */
	public DriveTrain() {
		super(left, right);

		//encoderL = new Encoder(Constants.DRIVE_ENCODER_LA, Constants.DRIVE_ENCODER_LB, false, EncodingType.k4X);
		//encoderL.setDistancePerPulse(DISTANCE_PER_PULSE_L);
		//encoderL.setSamplesToAverage(10);
		//encoderR = new Encoder(Constants.DRIVE_ENCODER_RA, Constants.DRIVE_ENCODER_RB, true, EncodingType.k4X);
		//encoderR.setDistancePerPulse(DISTANCE_PER_PULSE_R);
		//encoderR.setSamplesToAverage(10);
		// heading = new Heading();
		gearboxSolenoid = new Solenoid(Constants.GEARBOX_PNU);

		//Not sure if desired, was in example
		frontL.restoreFactoryDefaults();
		frontR.restoreFactoryDefaults();
		backL.restoreFactoryDefaults();
		backR.restoreFactoryDefaults();

		encoderL = new CANEncoder(frontL);
		encoderR = new CANEncoder(frontR);

		frontRPID = frontR.getPIDController();
		//pidL = new PID(0.005, 0, 0, false, true, "velL");
		//pidR = new PID(0.005, 0, 0, false, true, "velR");

		left.setInverted(false);
		right.setInverted(false);

		instance = this;
		shiftUp();
	}

	/**
	 * Shifts the drivetrain gearbox to high gear.
	 */

	public void shiftUp() {
		gearboxSolenoid.set(false);
		highGearEncoder();
		inHighGear = true;
	}

	private void highGearEncoder() {
		encoderL.setPositionConversionFactor(this.highGearCPI);
		encoderR.setPositionConversionFactor(this.highGearCPI);
	}

	/**
	 * Shifts the drivetrain gearbox to low gear.
	 */
	public void shiftDown() {
		gearboxSolenoid.set(true);
		lowGearEncoder();
		inHighGear = false;
	}

	private void lowGearEncoder() {
		encoderL.setPositionConversionFactor(this.lowGearCPI);
		encoderR.setPositionConversionFactor(this.lowGearCPI);
	}

	public void toggleShift() {
		if (isShiftedHigh()) {
			shiftDown();
			Common.debug("In Low Gear");
		} else {
			shiftUp();
			Common.debug("In High Gear");
		}
	}

	/**
	 * Whether or not the drivetrain is in high gear.
	 * 
	 * @return - is high
	 */
	public boolean isShiftedHigh() {
		// return shifter.get();
		return inHighGear;
	}

	/**
	 * Resets the counts of the left and right encoders.
	 */
	//public void resetEncoders() {
	//	encoderL.reset();
	//	encoderR.reset();
	//	pidL.reset();
	//	pidR.reset();
	//}

	/**
	 * Update PID tuning values from the SmartDashboard.
	 */
	//public void updatePIDs() {
		//pidL.update();
		//pidR.update();
	//}

	/**
	 * Returns an instance of DriveTrain which is bound to the motor controllers.
	 * Only this instance will be functional.
	 * 
	 * @return the DriveTrain instance.
	 */
	public static DriveTrain instance() {
		return instance;
	}

	/**
	 * Get raw counts for the left encoder.
	 * 
	 * @return the counts
	 */
	public double getLeftCounts() {
		return encoderL.getPosition();
	}

	/**
	 * Get the scaled distance of the left encoder.
	 * 
	 * @return the distance in inches
	 */
	public double getLeftDist() {
		return encoderL.getPosition();
	}

	/**
	 * Get the scaled velocity of the left encoder.
	 * 
	 * @return the velocity in inches/second
	 */
	public double getLeftVelocity() {
		return encoderL.getVelocity();
	}

	/**
	 * Get raw counts for the right encoder.
	 * 
	 * @return the counts
	 */
	public double getRightCounts() {
		return encoderR.getPosition();
	}

	/**
	 * Get the scaled distance of the right encoder.
	 * 
	 * @return double - the distance in inches
	 */
	public double getRightDist() {
		return encoderR.getPosition();
	}

	/**
	 * Get the scaled velocity of the right encoder.
	 * 
	 * @return double - the velocity in inches/second
	 */
	public double getRightVelocity() {
		return encoderR.getVelocity();
	}

	/**
	 * Get the averaged counts between the two encoders.
	 * 
	 * @return double - the average counts
	 */
	public double getAverageCounts() {
		return (getLeftDist() + getRightDist()) / 2.0;
	}

	/**
	 * Get the averaged scaled distance between the two encoders.
	 * 
	 * @return double - the average distance in inches
	 */
	public double getAverageDist() {
		return (getLeftDist() + getRightDist()) / 2;
	}

	/**
	 * Get the averaged scaled velocity between the two encoders.
	 * 
	 * @return double - the average velocity in inches/second
	 */
	public double getAverageVelocity() {
		return (getLeftVelocity() + getRightVelocity()) / 2;
	}

	/**
	 * An instance of Heading, a gyro utility and PID controller.
	 * 
	 * @return Heading - the heading instance.
	 */
	//public Heading getHeading() {
	//	return this.heading;
	//}

	/**
	 * Gets the drive acceleration value based on the elevator height and gear.
	 * 
	 * @return - the drive acceleration value
	 */

	/**
	 * Gradually accelerate to a specified drive value.
	 * 
	 * @param target - the target drive value from -1 to 1
	 * @return double - the allowed drive value for this cycle.
	 */
	public double driveAccelCurve(double target) {
		// nullzone

		if (Math.abs(target) < DEADZONE) {
			target = 0;
		}
		// If the magnitude of current is greater than the minimum
		// If the difference is greater than the allowed acceleration

		if (Math.abs(driveSpeed - target) > DRIVEACCEL) {
			// Accelerate in the correct direction
			if (driveSpeed > target) {
				driveSpeed = driveSpeed - DRIVEACCEL;
			} else {
				driveSpeed = driveSpeed + DRIVEACCEL;
			}
		} else {

			driveSpeed = target;
		}
		// If the magnitude of current speed is less than the minimum
		// Move to the greater of the minimum or the driveSpeed.
		if (Math.abs(target) > 0 && Math.abs(driveSpeed) < DRIVEMIN) {
			if (target > 0) {
				driveSpeed = DRIVEMIN;
			} else {
				driveSpeed = -DRIVEMIN;
			}
		}

		return driveSpeed;
	}

	/**
	 * Gradually accelerate to a specified turn value.
	 * 
	 * @param target - the target turn value from -1 to 1
	 * @return double - the allowed turn value at this cycle.
	 */
	public double turnAccelCurve(double target) {
		if (Math.abs(turnSpeed - target) > TURNACCEL) {
			if (turnSpeed > target) {
				turnSpeed = turnSpeed - TURNACCEL;
			} else {
				turnSpeed = turnSpeed + TURNACCEL;
			}
		} else {
			turnSpeed = target;
		}

		if (turnSpeed >= 0) {
			turnSpeed = Math.min(TURNMAX, turnSpeed);
		} else {
			turnSpeed = Math.max(-TURNMAX, turnSpeed);
		}
		return turnSpeed;
	}

	// turn should be inverted on testbed -Brent
	/**
	 * Arcade drive with an acceleration curve.
	 * 
	 * @param drive - the forward/backward value from -1 to 1.
	 * @param turn  - the turn left/right value from -1 to 1.
	 */
	public void accelDrive(double drive, double turn) {
		drive = driveAccelCurve(drive);
		turn = turnAccelCurve(turn);
		SmartDashboard.putNumber("Power: ", drive);
		arcadeDrive(drive, -turn);
	}

	/**
	 * An implementation of tank drive that updates current speed values used in
	 * acceleration curve methods. Does not set motors.
	 */
	@Override
	public void tankDrive(double left, double right) {
		tankLeft = left;
		tankRight = right;
	}

	/**
	 * Acceleration control for tank drive. Does not set motors.
	 * 
	 * @param left  - the target left power.
	 * @param right - the target right power.
	 */
	public void accelTankDrive(double left, double right) {
		tankLeft = accelSide(tankLeft, left);
		tankRight = accelSide(tankRight, right);

		// System.out.println(tankLeft + ":" + tankRight);
	}

	/**
	 * Applies the current DriveTrain tankLeft and tankRight motor powers. Uses
	 * heading hold PID if heading hold is enabled with
	 * {@link Heading#setHeadingHold(boolean)}
	 */
	//public void applyTankDrive() {
		//if (heading.isHeadingHold()) {
			//double turn = heading.turnRate() / 2;
			//.tankDrive(tankLeft + turn, tankRight - turn);
		//} else {
			//super.tankDrive(tankLeft, tankRight);
		//}
	//}

	/**
	 * A utility method that determines the change in current power, given a desired
	 * target and allowed power curve.
	 * 
	 * @param current the current motor power.
	 * @param target  the target motor power.
	 * @return the new motor power.
	 */
	private double accelSide(double current, double target) {
		// If the magnitude of current is less than the minimum
		if (Math.abs(current) < TANKMIN) {
			// Move to the lesser value of the minimum or the target, including desired
			// direction.
			if (target > 0) {
				current = Math.min(TANKMIN, target);
				current = Math.max(TANKMAX, target);
			} else {
				current = Math.max(-TANKMIN, target);
				current = Math.min(-TANKMAX, target);
			}
		}
		// If the magnitude of current is greater than the minimum
		// If the difference is greater than the allowed acceleration
		if (Math.abs(current - target) > TANKACCEL) {
			// Accelerate in the correct direction
			if (current > target) {
				current = current - TANKACCEL;
			} else {
				current = current + TANKACCEL;
			}
		}
		// If the difference is less than the allowed acceleration, reach target
		else {
			current = target;
		}
		return current;
	}
	public void debug() {
		Common.dashNum("Front encoder counts", getRightCounts());

		Common.dashNum("P Gain", P);
    	Common.dashNum("I Gain", I);
		Common.dashNum("D Gain", D);
    	//SmartDashboard.putNumber("I Zone", I);
    	Common.dashNum("Feed Forward", FF);
    	Common.dashNum("Max Output", maxOutput);
		Common.dashNum("Min Output", minOutput);
		double p = Common.getNum("P Gain", 0);
		double i = Common.getNum("I Gain", 0);
		double d = Common.getNum("D Gain", 0);
		double iz = Common.getNum("I Zone", 0);
		double ff = Common.getNum("Feed Forward", 0);
		double max = Common.getNum("Max Output", 0);
		double min = Common.getNum("Min Output", 0);
		if((p != P)) { frontRPID.setP(p); P = p; }
    	if((i != I)) { frontRPID.setI(i); I = i; }
    	if((d != D)) { frontRPID.setD(d); D = d; }
    	//if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    	if((ff != FF)) { frontRPID.setFF(ff); FF = ff; }
    	if((max != maxOutput) || (min != minOutput)) { 
			frontRPID.setOutputRange(min, max); 
     		minOutput = min; maxOutput = max; 
    	}
	}
}
