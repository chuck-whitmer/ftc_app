package org.firstinspires.ftc.teamcode;

public class AutoPath {
    static int[] programRightGold = new int[]
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

    static int[] programLeftGold = new int[]
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

    static int[] programCenterGold = new int[]
            {
                    HardwareCwBot.SETHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(57.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-7.0),
                    HardwareCwBot.TURNTOHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(11.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(59.0),
            };

    static int[] programToAndFro = new int[]
            {
                    HardwareCwBot.SETHEADING, 0,
                    HardwareCwBot.APPROACHTO, 40,
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.APPROACHTO, 70,
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.APPROACHTO, 40,
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.APPROACHTO, 70,
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.APPROACHTO, 40,
                    HardwareCwBot.TURNTOHEADING, 0,
            };

    static int[] programOrbit = new int[]
            {
                    HardwareCwBot.SETHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.TURNTOHEADING, -135,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(34.5),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.APPROACHTO, 36,

                    HardwareCwBot.TURNTOHEADING, 135,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(69.0),
                    HardwareCwBot.TURNTOHEADING, 180,
                    HardwareCwBot.APPROACHTO, 36,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(69.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.APPROACHTO, 36,

                    HardwareCwBot.TURNTOHEADING, 135,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(69.0),
                    HardwareCwBot.TURNTOHEADING, 180,
                    HardwareCwBot.APPROACHTO, 36,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(69.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.APPROACHTO, 36,

                    HardwareCwBot.TURNTOHEADING, -135,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-34.5),
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-13.0),
                    HardwareCwBot.SETPOWER, 50,
//                    HardwareCwBot.TURNTOHEADING, 45,
//                    HardwareCwBot.DRIVE, HardwareCwBot.inches(68.0),
//                    HardwareCwBot.TURNTOHEADING, 0,
//                    HardwareCwBot.DISPLAY,0,
//                    HardwareCwBot.APPROACHTO, 36,
//                    HardwareCwBot.TURNTOHEADING, -135,
//                    HardwareCwBot.DRIVE, HardwareCwBot.inches(68.0),

            };


    static int[] programCraterRightGold = new int[]
            {
                    HardwareCwBot.SETHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(14.5),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-13.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(45.5),
                    HardwareCwBot.TURNTOHEADING, 0,
                    HardwareCwBot.APPROACHTO, 36,
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(18.0),
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(30.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(10.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(9.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-6.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-4.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(73.0)
            };

    static int[] programCraterCenterGold = new int[]
            {
                    HardwareCwBot.SETHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-12.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(45.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(49.5),
                    HardwareCwBot.WAIT, 500,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-6.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-3.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-3.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(68.0)
            };

    static int[] programCraterLeftGold = new int[]
            {
                    HardwareCwBot.SETHEADING, 45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(13.0),
                    HardwareCwBot.SETPOWER, 25,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-14.5),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(12.0),
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-12.0),
                    HardwareCwBot.SETPOWER, 50,
                    HardwareCwBot.TURNTOHEADING, -45,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(41.0),
                    HardwareCwBot.TURNTOHEADING, -90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(49.5),
                    HardwareCwBot.WAIT, 500,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(-6.0),
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-3.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.STRAFE, HardwareCwBot.inches(-3.0),
                    HardwareCwBot.TURNTOHEADING, 90,
                    HardwareCwBot.DRIVE, HardwareCwBot.inches(68.0)
            };


}
