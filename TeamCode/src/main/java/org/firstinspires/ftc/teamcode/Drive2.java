package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.MecanumDrive;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;


import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp(name="Drive2", group="Linear Opmode")
public class Drive2 extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();


    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;


    private DcMotor armDrive = null;
    private DcMotor armSlide = null;
    private DcMotor leftSlide = null;
    private DcMotor rightSlide = null;

    Servo gripper;
    Servo gripperAngle;


    //DistanceSensor distance;


    public boolean gripperAngleUp;
    public boolean gripperClosed;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));


        frontLeftDrive  = hardwareMap.get(DcMotor.class, "Front Left");          //TODO: Correct naming
        frontRightDrive = hardwareMap.get(DcMotor.class, "Front Right");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "Back Left");
        backRightDrive = hardwareMap.get(DcMotor.class, "Back Right");


        armDrive = hardwareMap.get(DcMotor.class, "Arm");
        armSlide = hardwareMap.get(DcMotor.class, "Arm Slide");

        rightSlide = hardwareMap.get(DcMotor.class, "Right Slide");
        leftSlide = hardwareMap.get(DcMotor.class, "Left Slide");

        gripperAngle = hardwareMap.get(Servo.class, "Gripper Angle");
        gripper = hardwareMap.get(Servo.class, "Gripper");


        //distance = hardwareMap.get(DistanceSensor.class, "sensor_distance");


        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);


        frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        armDrive.setDirection(DcMotor.Direction.FORWARD);
        armSlide.setDirection(DcMotor.Direction.FORWARD);


        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");  //
        telemetry.update();


        frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Path0",  "Starting at %7d", armDrive.getCurrentPosition());
        telemetry.update();


        boolean gripperPreviouslyPressed = false;
        boolean gripperAnglePreviouslyPressed = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        boolean gripperAngleButtonOn = false;
        boolean gripperButtonOn = false;

       /*gripperAngle.setPosition(0.345);//Quick fix cuz uhhhhh
       gripper.setPosition(0.95); //close
       gripperClosed=false;
       gripperAngleUp=true;*/

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ),
                    -gamepad1.right_stick_x
            ));

            //Arm Controls with dpad
            if ((gamepad2.dpad_right == true)) {//&& (topLimit.getState()== true)){
                //third level
                armUp();
            }else{
                if ((gamepad2.dpad_left == true)) {//&& (lowLimit.getState() == true)){
                    //arm go down
                    armDown();
                }else{
                    //if ((gamepad2.dpad_up == true) && (topLimit.getState() == true) && (armDrive.getCurrentPosition() > capEncoder)){
                    if ((gamepad2.dpad_right == true)){// && (topLimit.getState() == true)){
                        //arm go up
                        armUp();
                    }else{
                        //arm stops
                        armStop();
                    }
                }
            }


            //Arm Slide Control with le bumpahs
            if ((gamepad2.right_bumper == true)) {
                //Arm slide slides forth
                armSlideForth();
            }else{
                if ((gamepad2.left_bumper == true)) {
                    //Arm slide slides back
                    armSlideBack();
                }else{
                    if ((gamepad2.right_bumper == true)){
                        armUp();
                    }else{
                        armSlideStop();
                    }
                }
            }

            //Slides up/down with D-Pad
            if (gamepad2.dpad_up) {
                slidesUp();
            }else{
                if (gamepad2.dpad_down) {
                    slidesDown();
                }else{
                    slidesStop();
                }}


                //Gripper closes/opens with X
                if (gamepad2.x==true){
                    if (!gripperPreviouslyPressed) { gripperSwitch(); }
                    gripperPreviouslyPressed=true;
                } else { gripperPreviouslyPressed = false; }

                //Gripper angle cycles up/down with A
                if (gamepad2.a==true){
                    if (!gripperAnglePreviouslyPressed) { gripperAngleSwitch(); }
                    gripperAnglePreviouslyPressed=true;
                } else { gripperAnglePreviouslyPressed = false; }


                //Display Information
                telemetry.addData("Path0",  "ArmDrive at %7d",armDrive.getCurrentPosition());
                telemetry.addData("Path0",  "Front RightF at %7d",frontRightDrive.getCurrentPosition());
                telemetry.addData("Path0",  "Front Left at %7d",frontLeftDrive.getCurrentPosition());
                telemetry.addData("Path0",  "Back Right at %7d",backRightDrive.getCurrentPosition());
                telemetry.addData("Path0",  "Back Left at %7d",backLeftDrive.getCurrentPosition());
                telemetry.addData("Gripper left", String.format("position=%.2f", gripper.getPosition()));
                //telemetry.addLine("Distance Sensor at "+distance.getDistance(DistanceUnit.CM));
                telemetry.addLine("welcome to the future ");
                telemetry.update();


                //armCurrentPosition=armDrive.getCurrentPosition();
        }
    }


//*********
//functions
//*********

        public void slideRight() {
            frontLeftDrive.setPower(0.6);
            frontRightDrive.setPower(-0.6);
            backLeftDrive.setPower(-0.6);
            backRightDrive.setPower(0.6);
        }


        public void slideLeft() {
            frontLeftDrive.setPower(-0.6);
            frontRightDrive.setPower(0.6);
            backLeftDrive.setPower(0.6);
            backRightDrive.setPower(-0.6);
        }


        /*Arm Power*/
        public void armUp() {
            armDrive.setPower(-0.4); //was -0.15
        }
        public void armDown() {
            armDrive.setPower(0.4);//was 0.15
        }
        public void armStop(){
            armDrive.setPower(0);// stop arm
        }


        /*Arm Sliding*/
        public void armSlideForth() {
            armSlide.setPower(1.0);
        }
        public void armSlideBack() {
            armSlide.setPower(-1.0);
        }
        public void armSlideStop() {
            armSlide.setPower(0);
        }

        /*Open/close gripper*/
        public void gripperSwitch() {
            if (gripperClosed==true) {
                gripper.setPosition(0.95); //close
                gripperClosed=false;
            }
            else {
                gripper.setPosition(0.1); //open
                gripperClosed=true;
            }
        }

        /*Servo arm go up/down*/
        public void gripperAngleSwitch() {
            if (gripperAngleUp==true) {
                gripperAngle.setPosition(0.345); //go down
                gripperAngleUp=false;
            }
            else {
                gripperAngle.setPosition(0);   //go up
                gripperAngleUp=true;
            }
        }

        /*Slides Up/Down*/
        public void slidesUp() {
            rightSlide.setPower(0.5);
            leftSlide.setPower(0.5);
        }
        public void slidesDown() {
            rightSlide.setPower(-0.1);
            leftSlide.setPower(-0.1);
        }
        public void slidesStop() {
            rightSlide.setPower(0);
            leftSlide.setPower(0);
        }
}