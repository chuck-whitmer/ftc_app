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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

@Autonomous(name="Depot Side", group="cwbot")
public class cwbotAutoDepot extends LinearOpMode
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
        telemetry.addData("Say", "Hello Otto");    //
        telemetry.update();

        // Wait for the game to start (drive r presses PLAY)
        waitForStart();
        robot.resetTickPeriod();

        robot.RunProgram(programRightGold, this);
    }

    int[] programRightGold = new int[]
            {
                    HardwareCwBot.SETHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(19.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(16.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-6.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-3.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(58.0)
            };

    int[] programLeftGold = new int[]
            {
                    HardwareCwBot.SETHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(18.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(16.0),
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-6.0),
                    HardwareCwBot.TURNTOHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(18.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-4.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(59.5),
            };

    int[] programCenterGold = new int[]
            {
                    HardwareCwBot.SETHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(57.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-7.0),
                    HardwareCwBot.TURNTOHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(11.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(59.0),
            };

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

    int[] programTestTurns = new int[]
            {
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0),
                    HardwareCwBot.TURN, HardwareCwBot.degrees(90.0)
            };
}
