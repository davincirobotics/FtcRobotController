/* 
Nov.25: autonomous was born by a piece of the
		flesh of Drive1.java
Nov.26: A technical prototype of the sequence is done now
14.Nov.2024: we do a little recycling lol
 */

package firstinspires.firstinspires.ftc.centerstage;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.hardware.bosch.BNO055IMU;

@Autonomous(name="ParkObservation", group="Linear Opmode")

public class ParkObservation extends LinearOpMode {

	// Declare OpMode members.
	private ElapsedTime runtime = new ElapsedTime();
	
	private DcMotor frontLeftDrive = null;
	private DcMotor frontRightDrive = null;
	private DcMotor backLeftDrive = null;
	private DcMotor backRightDrive = null; 
	
	private DcMotor armDrive = null; 

	Servo gripper;	
	Servo gripperAngle;
	
	public boolean gripperAngleUp;
	public boolean gripperClosed;

	DistanceSensor distance;

	public double robotSpeed=0.5;

	public double encoderPulses; //number of ticks needed to move
	public double wheelCircumference = 30.159; //circumference of wheel
	public double motorResolution = 537.7;	//motor res

	BNO055IMU imu;
	Orientation angles;
	
	@Override
	public void runOpMode() {
		telemetry.addData("Status", "Initialized");
		telemetry.update();

		// Initialize the hardware variables. Note that the strings used here as parameters
		// to 'get' must correspond to the names assigned during the robot configuration
		// step (using the FTC Robot Controller app on the phone).
		frontLeftDrive  = hardwareMap.get(DcMotor.class, "Front Left");
		frontRightDrive = hardwareMap.get(DcMotor.class, "Front Right");
		backLeftDrive  = hardwareMap.get(DcMotor.class, "Back Left");
		backRightDrive = hardwareMap.get(DcMotor.class, "Back Right");
	
		armDrive = hardwareMap.get(DcMotor.class, "Arm");
		
		gripperAngle = hardwareMap.get(Servo.class, "Gripper Angle");
		gripper = hardwareMap.get(Servo.class, "Gripper");

		//distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");
		
		frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
		backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
		backRightDrive.setDirection(DcMotor.Direction.FORWARD);
		
		armDrive.setDirection(DcMotor.Direction.REVERSE);

		BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
		parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
		parameters.calibrationDataFile = "BNO055IMUCalibration.json";
		
		imu = hardwareMap.get(BNO055IMU.class, "imu");
		imu.initialize(parameters);
		
		armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		
		// Send telemetry message to signify robot waiting;
		telemetry.addData("Status", "Resetting Encoders");	//
		telemetry.update();

		// Wait for the game to start (driver presses PLAY)
		waitForStart();		
		runtime.reset();
		
		// run until the end of the autonomous mode (thirty seconds pass)
		while (opModeIsActive()) {
			
			//gripper.setPosition(0.95); //close
			//gripper2.setPosition(0.05); //close
			//telemetry.addLine("closed gripper");
			//telemetry.update();
			//sleep(1000);
			//gripperAngle.setPosition(0.345); //gripper angle go down to floor level
			//telemetry.addLine("lowered gripper");
			//telemetry.update();
			
                       // rotateVer2(90,1,0.25);
			moveForwardCM(10.0);
			sleep(500);
		}
	}
		
//*********		
//functions
//*********
	public void rotateVer2(double angleIn, double angleTol, double speedIn){
			double angleMin;
			double angleMax;
			
			angleMin = (angleIn - angleTol);
			angleMax = (angleIn + angleTol);
			
			angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
			while ((angles.firstAngle < angleMin) || (angles.firstAngle > angleMax )){
				//out of target
				if (angles.firstAngle < angleMin) {
					//rotate CCW
					if (Math.abs(angles.firstAngle - angleIn) < 20) {
						frontLeftDrive.setPower(-speedIn/4);
						frontRightDrive.setPower(speedIn/4);
						backLeftDrive.setPower(-speedIn/4);
						backRightDrive.setPower(speedIn/4);   
					}else{
						frontLeftDrive.setPower(-speedIn);
						frontRightDrive.setPower(speedIn);
						backLeftDrive.setPower(-speedIn);
						backRightDrive.setPower(speedIn);	
					}
				}
				if (angles.firstAngle > angleMax) {
					//rotate CW
					if (Math.abs(angles.firstAngle - angleIn) < 20){
						frontLeftDrive.setPower(speedIn/4);
						frontRightDrive.setPower(-speedIn/4);
						backLeftDrive.setPower(speedIn/4);
						backRightDrive.setPower(-speedIn/4);   
					}else{
						frontLeftDrive.setPower(speedIn);
						frontRightDrive.setPower(-speedIn);
						backLeftDrive.setPower(speedIn);
						backRightDrive.setPower(-speedIn);  
					}
				}
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
			}
				stopWheels();
			
				sleep(1000);
				angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES );
				telemetry.addData("Angles", "First (%.2f), Second (%.2f), Third (%.2f) ", angles.firstAngle, angles.secondAngle, angles.thirdAngle);
				telemetry.update();   
	}

	public void moveForward() {
			frontLeftDrive.setPower(robotSpeed);
			frontRightDrive.setPower(robotSpeed);
			backLeftDrive.setPower(robotSpeed);
			backRightDrive.setPower(robotSpeed);
	}
	public void moveBackwards() {
		frontLeftDrive.setPower(-robotSpeed);
		frontRightDrive.setPower(-robotSpeed);
		backLeftDrive.setPower(-robotSpeed);
		backRightDrive.setPower(-robotSpeed);
	}
	public void stopWheels() {
		frontLeftDrive.setPower(0);
		frontRightDrive.setPower(0);
		backLeftDrive.setPower(0);
		backRightDrive.setPower(0);
	}
	
	public void moveForwardCM(double distance){
			frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			encoderPulses=motorResolution*(distance/wheelCircumference); //formula to calculate the encoder pulses needed to move
			moveForward(); //moves forth
			while ((frontRightDrive.getCurrentPosition() <= encoderPulses) || (backLeftDrive.getCurrentPosition() <= encoderPulses) || (frontLeftDrive.getCurrentPosition() <= encoderPulses) || (backRightDrive.getCurrentPosition() <= encoderPulses)){
				//stay here until distance is the needed one
			}
			stopWheels();
	}

	public void moveBackCM(double distance){
			frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
			encoderPulses=motorResolution*(distance/wheelCircumference); //formula to calculate the encoder pulses needed to move
			moveBackwards(); //moves forth
			while ((frontRightDrive.getCurrentPosition() >= -encoderPulses) || (backLeftDrive.getCurrentPosition() >= -encoderPulses) || (frontLeftDrive.getCurrentPosition() >= -encoderPulses) || (backRightDrive.getCurrentPosition() >= -encoderPulses)){
				//stay here until distance is the needed one
			}
			stopWheels();
	}

	/*Arm Power*/
	public void armUp() {
		armDrive.setPower(0.2); //was -0.8
	}
	public void armDown() {
		armDrive.setPower(-0.2);//was 1.0
	}
	public void armStop(){
		armDrive.setPower(0);// stop arm
	}
	
	public void armForDroppingYellowPixel() {
		armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		armUp();
		while (armDrive.getCurrentPosition()<750){
			//stay here for a "while"
		}
		armStop();
	}
}
