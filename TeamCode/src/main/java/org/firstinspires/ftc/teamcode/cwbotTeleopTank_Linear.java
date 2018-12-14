/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="cwbot: Teleop Drive", group="cwbot")
public class cwbotTeleopTank_Linear extends LinearOpMode
{
    /* Declare OpMode members. */
    HardwareCwBot robot = new HardwareCwBot();

    @Override
    public void runOpMode() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (drive r presses PLAY)
        waitForStart();
        robot.resetTickPeriod();

        boolean aLastState = false;
        boolean aPressed = false;
        boolean bLastState = false;
        boolean bPressed = false;
        boolean yLastState = false;
        boolean yPressed = false;
        boolean xLastState = false;
        boolean xPressed = false;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //robot.backRight.setPower(gamepad1.right_bumper ? 1.0 : 0.0);
            //robot.backLeft.setPower(gamepad1.left_bumper ? 1.0 : 0.0);


            float x = gamepad1.left_stick_x;
            float y = -gamepad1.left_stick_y; // Negate to get +y forward.
            float rotation = gamepad1.right_stick_x;
            float speedControl = 0.5f*(1.0f + gamepad1.left_trigger);
            double biggestControl = Math.sqrt(x*x+y*y);
            double biggestWithRotation = Math.sqrt(x*x+y*y+rotation*rotation);

            double angle = Math.atan2(y,-x) - Math.PI/2.0;

            double[] powers = robot.getDrivePowersFromAngle(angle);
            double pow2 = 0.0;
            for (int i=0; i<robot.allMotors.length; i++)
            {
                double pow = powers[i]*biggestControl + rotation * robot.turnFactors[i];
                powers[i] = pow;
                pow2 += pow*pow;
            }

            if (biggestWithRotation != 0.0) {
                double scale = Math.sqrt(pow2);
                for (int i = 0; i < robot.allMotors.length; i++) {
                    robot.allMotors[i].setPower(
                            powers[i]/scale*biggestWithRotation*speedControl);
                }
            }
            else
            {
                for (int i = 0; i < robot.allMotors.length; i++)
                    robot.allMotors[i].setPower(0.0);
            }

//            if (gamepad1.right_bumper)
//                TestAutoR();
//            if (gamepad1.left_bumper)
//                TestAutoL();

            aPressed = gamepad1.a && !aLastState;
            aLastState = gamepad1.a;
            bPressed = gamepad1.b && !bLastState;
            bLastState = gamepad1.b;
            yPressed = gamepad1.y && !yLastState;
            yLastState = gamepad1.y;
            xPressed = gamepad1.x && !xLastState;
            xLastState = gamepad1.x;

            // YBA = PID
            if (yPressed)
            {
                TestAutoR();
            }

            if (bPressed)
            {
                TestAutoC();
            }

            if (xPressed)
            {
                robot.runWithHeadingKp = 0.0;
            }

            int encoderA = robot.frontLeft.getCurrentPosition();
            int encoderB = robot.backLeft.getCurrentPosition();
            int encoderC = robot.frontRight.getCurrentPosition();
            int encoderD = robot.backRight.getCurrentPosition();

            //Quaternion q = robot.imu.getQuaternionOrientation();
            //telemetry.addData("Q", "%.5f %.5f %.5f %.5f",q.w,q.x,q.y,q.z);
            telemetry.addData("heading", "%.1f",robot.getHeading());
            telemetry.addData("Encoders","%6d %6d %6d %6d", encoderA,encoderB,encoderC,encoderD);
//            telemetry.addData("PID", "%.5f %.5f %.5f",robot.runWithHeadingKp,robot.runWithHeadingKi,robot.runWithHeadingKd);
            // The sonar only refreshes at 6.7 Hz.
            // We will average over 1 second to reduce noise.
            double dLeft = robot.getFrontDistance();
            double dRight = 0.0; // robot.rev2M.getDistance(DistanceUnit.CM);
            telemetry.addData("ds",  "%.2f %.2f", dLeft, dRight);
            telemetry.update();

            // Pause for 40 mS each cycle = update 25 times a second.
            //sleep(40);
            robot.waitForTick(40);
        }
    }

    int[] programTestDrive = new int[]
            {
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0)
            };
    int[] programTestStrafe = new int[]
            {
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(12.0)
            };

    void TestAutoR()
    {
        robot.RunProgram(AutoPath.programOrbit,this);
    }
    void TestAutoC()
    {
        robot.RunProgram(AutoPath.programToAndFro,this);
    }

    int[] programTestTurns = new int[]
            {
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0)
            };

    void TestAutoL()
    {
        robot.RunProgram(AutoPath.programLeftGold,this);
    }
}
