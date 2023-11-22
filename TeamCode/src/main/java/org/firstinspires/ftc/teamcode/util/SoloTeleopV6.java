package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



/* v6 solo Teleop !!!!!!!!!!!!!

Controls:

Drive mode: robot Centric
left stick: strafe
right stick: rotation

B: Toggle Intake (in) *pressing out while intaking will switch to out and vice versa (see intake fsm)
A: Toggle Intake (out)

X: extend to last position/retract lift
dpadUp: extend to high target
dpadRight: extend to medium target
dpadLeft: extend to low target
Y: (only when fully extended) (once) ready deposit (only when fully extended), (twice) release pixels, (thrice) reset deposit and lift

right bumper: adjust lift upwards
lift bumper: adjust lift downwards

start: toggle slow mode

 */


@TeleOp
public class SoloTeleopV6 extends LinearOpMode {
    public enum LiftState {
        START,
        EXTEND,
        DUMP,
        RETRACT,
        DEPOSIT,
        RESETSERVO,
        RESETSERVO2;
    };
    public enum IntakeState {
        IDLE,
        IN,
        OUT;
    }
    // Current state of our lift to avoid collisions and
    // make it easier on the driver
    LiftState liftState = LiftState.START;
    IntakeState intakeState = IntakeState.IDLE;

    public double controllerCurve(double value){ // exponential curve for fine control
        // return Math.pow(value, 2); // uncomment line if you want this i guess
        return value; // just empty for now
    }
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime xTimer = new ElapsedTime();
        ElapsedTime aTimer = new ElapsedTime();
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("BackLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("FrontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("BackRight");

        Servo dumpServo = hardwareMap.servo.get("flipBucket");
        Servo holdServo = hardwareMap.servo.get("pixelRelease");
        Servo intakeServo = hardwareMap.servo.get("IntakeServoLeft");

        //Servo leftIntakeServo= hardwareMap.servo.get("bao");
        //Servo rightIntakeServo= hardwareMap.servo.get("bao");


        DcMotor lift = hardwareMap.dcMotor.get("Lift");
        lift.setDirection(DcMotor.Direction.REVERSE);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //holdServo.setPosition(0.25);
        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DcMotor intake = hardwareMap.dcMotor.get("Intake");

        boolean fieldCentric = false;
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        /*
        // WE USING THE IMU WE UP WE UP WE UP WE UP
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // bruh idk what our robot looks like
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        */
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //lift.setPower(0.5);
            double y = -(controllerCurve(gamepad1.left_stick_y)) * 1; // Remember, Y stick value is reversed
            double x = (controllerCurve(gamepad1.left_stick_x)) * 1.1; // Counteract imperfect strafing
            double rx = (controllerCurve(gamepad1.right_stick_x)) * 1;

            // robot centric drive
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); // OMG MATH MATH MATH MATH MATH MATH
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // field centric drive
            /*
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;
            double FSdenominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double FSfrontLeftPower = (rotY + rotX + rx) / denominator;
            double FSbackLeftPower = (rotY - rotX + rx) / denominator;
            double FSfrontRightPower = (rotY - rotX - rx) / denominator;
            double FSbackRightPower = (rotY + rotX - rx) / denominator;
            */
            double slowFactor = 0.1;
            double FSy = (slowFactor * y); // Remember, Y stick value is reversed
            double FSx = (slowFactor * x); // Counteract imperfect strafing
            double FSrx = (slowFactor * rx);
            double FSdenominator = Math.max(Math.abs(FSy) + Math.abs(FSx) + Math.abs(FSrx), 1);
            double FSfrontLeftPower = (FSy + FSx + FSrx) / FSdenominator;
            double FSbackLeftPower = (FSy - FSx + FSrx) / FSdenominator;
            double FSfrontRightPower = (FSy - FSx - FSrx) / FSdenominator;
            double FSbackRightPower = (FSy + FSx - FSrx) / FSdenominator;
            // lift config values
            int liftHigh = 2000; // highest point on lift
            int liftLow = 600; // lowest point lift will go manually to prevent deposit hitting intake
            int liftTarget = 1500; // target position, starts at 1500

            int lowTarget = 700; // low Target position for lift
            int midTarget = 1500; // medium target for lift
            int highTarget = 2000; // high target for lift

            // deposit config values
            double depositAngle = 0.35; // how far the deposit will turn when depositing
            double hold0 = 0.25; // bucket holding 0 pixles
            double hold1 = 0.25; // bucket holding 1 pixel
            double hold2 = 0.10; // bucket holding 2 pixels

            // timer and error correction values
            int depositResetTime = 1; // the time for our deposit servo to reset before retracting
            int msDelay = 1; // delay between button inputs, because there's a smarter way but i'm way too lazy
            int intakeDelay = 1; // delay between intake toggles, just different because I kind of like it different
            int tick_error = 10; // maximum error allowed for our lift position

            // intake config values
            double leftIntakeIdle = 1;
            double leftIntakeActive = 0.75;
            double rightIntakeIdle = 0.25;
            double rightIntakeActive = 0;
            double intakePower = 1;
            double rejectPower = -1;

            // lift code OMG FSM FSM FSM FSM FSM FSM FSM
            switch (liftState){
                case START:
                    if (gamepad1.x && xTimer.time() > msDelay){ // goes back to previous lift target
                        xTimer.reset();
                        holdServo.setPosition(hold2);
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        liftState = LiftState.EXTEND;
                    }
                    else if (gamepad1.dpad_up && xTimer.time() > msDelay){ // goes to high lift target
                        xTimer.reset();
                        holdServo.setPosition(hold2);
                        liftTarget = highTarget;
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        liftState = LiftState.EXTEND;
                    }
                    else if (gamepad1.dpad_right && xTimer.time() > msDelay){ // goes to medium lift target
                        xTimer.reset();
                        holdServo.setPosition(hold2);
                        liftTarget = midTarget;
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        liftState = LiftState.EXTEND;
                    }
                    else if (gamepad1.dpad_left && xTimer.time() > msDelay){ // goes to low lift target
                        xTimer.reset();
                        holdServo.setPosition(hold2);
                        liftTarget = lowTarget;
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(1);
                        liftState = LiftState.EXTEND;
                    }
                    else{
                        lift.setPower(0);
                        holdServo.setPosition(hold0);
                    }
                    break;
                case EXTEND:
                    holdServo.setPosition(hold2);
                    if (Math.abs(lift.getCurrentPosition() - liftTarget) <= tick_error){
                        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.1); // Might be needed, don't remember
                        liftState = LiftState.DUMP;
                    }
                    else if (gamepad1.x && xTimer.time() > msDelay){ // accidentally raise
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        xTimer.reset();
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.5);
                        liftState = LiftState.RETRACT;
                    }
                    break;
                case DUMP:
                    //holdServo.setPosition(hold2);
                    if (gamepad1.y  && xTimer.time() > msDelay){ // prepare to deposit
                        xTimer.reset();
                        dumpServo.setPosition(depositAngle);
                        liftState = LiftState.DEPOSIT;
                    }
                    else if (gamepad1.x && xTimer.time() > msDelay){ // accidentally raisee
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        xTimer.reset();
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.5);
                        liftState = LiftState.RETRACT;
                    }
                    else if (gamepad1.left_bumper){
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftTarget = Math.max(liftLow, liftTarget - 100);
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.5);
                        liftState = LiftState.EXTEND;
                    }
                    else if (gamepad1.right_bumper){
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftTarget = Math.min(liftHigh, liftTarget + 100);
                        lift.setTargetPosition(liftTarget);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0.5);
                        liftState = LiftState.EXTEND;
                    }
                    break;
                case DEPOSIT:
                    if (gamepad1.y && xTimer.time() > msDelay){
                        xTimer.reset();
                        holdServo.setPosition(hold0);
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftState = LiftState.RESETSERVO;
                    }
                    break;
                case RESETSERVO:
                    if (gamepad1.y && xTimer.time() > msDelay){
                        xTimer.reset();
                        dumpServo.setPosition(0);
                        // lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftState = LiftState.RESETSERVO2;
                    }
                    break;
                case RESETSERVO2: // sorry for the mistake here, this state has to be added so we don't exp    erience collisions
                    if(xTimer.time() > depositResetTime){ // wait for servo to reset, adjust later PLEASEEEEEEEEE
                        lift.setTargetPosition(0);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftState = LiftState.RETRACT;
                        lift.setPower(1);
                    }
                    else{
                        holdServo.setPosition(hold0);
                    }
                case RETRACT:
                    if (Math.abs(lift.getCurrentPosition()) <= 2){
                        //lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        lift.setPower(0); // Might be needed, don't remember
                        liftState = LiftState.START;
                    }
                    break;
                default:
                    liftState = LiftState.START;
            }


            // intake code
            switch(intakeState){ // we already have one FSM, why not ANOTHER??!??!?!??!?!?!?!?!
                case IDLE: // intake idle, what else would it be??????? idk why i write comments anymore :(((((((((((((
                    intake.setPower(0);
                    intakeServo.setPosition(leftIntakeIdle);
                    //rightIntakeServo.setPosition(rightIntakeIdle);
                    if (gamepad1.b && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.IN;
                    }
                    else if (gamepad1.a && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.OUT;
                    }
                    break;
                case IN: // intake pixel
                    intakeServo.setPosition(leftIntakeActive);
                    //rightIntakeServo.setPosition(rightIntakeActive);
                    intake.setPower(intakePower);
                    if (gamepad1.b && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.IDLE;
                    }
                    else if (gamepad1.a && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.OUT;
                    }
                    break;
                case OUT: // spit out pixel because driver's trash
                    intakeServo.setPosition(leftIntakeIdle);
                    //rightIntakeServo.setPosition(rightIntakeIdle);
                    intake.setPower(rejectPower);
                    if (gamepad1.b && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.IN;
                    }
                    else if (gamepad1.a && aTimer.time() > intakeDelay){
                        aTimer.reset();
                        intakeState = IntakeState.IDLE;
                    }
                    break;
            }

            if (gamepad1.start && aTimer.time() > 2000){aTimer.reset(); fieldCentric = !fieldCentric;} // toggle field centric driving

            // drive code (IF IT'S BUILT CORRECTLY AHHHHHHHHHHHHHHHHHHH)
            if (!fieldCentric){
                frontLeftMotor.setPower(frontLeftPower);
                backLeftMotor.setPower(backLeftPower);
                frontRightMotor.setPower(frontRightPower);
                backRightMotor.setPower(backRightPower);
            }
            else{ // actually slow mode rn
                frontLeftMotor.setPower(FSfrontLeftPower);
                backLeftMotor.setPower(FSbackLeftPower);
                frontRightMotor.setPower(FSfrontRightPower);
                backRightMotor.setPower(FSbackRightPower);
            }
        }
    }
}
/*
             /)  (\
        .-._((,~~.))_.-,
         `=.   oo   ,='
           / ,o~~o. \
          { { .__. } }
           ) `~~~\' (
          /`-._  _\.-\
         /         )  \
       ,-X        #   X-.
      /   \          /   \
     (     )| |  | |(     )
      \   / | |  | | \   /
       \_(.-( )--( )-.)_/
       /_,\ ) /  \ ( /._\
           /_,\  /._\
 */