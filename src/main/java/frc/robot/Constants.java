package frc.robot;

public class Constants {
    public class IDs {
        public class Shooter {
            public static final int bottomShootingMotorId = 0;
            public static final int topShootingMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }

        public class Intake {
            public static final int intakeMotorId = 0;
            public static final int leftWristMotorId = 0;
            public static final int rightWristMotorId = 0;
        }
    }

    public class PIDs {}

    public class Limelights {
        public static final String shooterLimelightName = "limelight";
        public static final String extraLimelightName = "limelight2";
        public class AprilTagLimits {
            public static final double XError = 0.0;
            public static final double ZError = 2;
            public static final double yawError = 0.0;
            public static final double ZDeadband = 0.05; //cm
        }
        public class PID {
            public class Rotation {
                public static final double kP = 0.1;
                public static final double kI = 0.05;
                public static final double kD = 0.0;
            }
            public class X {
                public static final double kP = 4.75;
                public static final double kI = 0.1;
                public static final double kD = 0.1;
            }
            public class Z {
                public static final double kP = 2.5;
                public static final double kI = 0.0;
                public static final double kD = 0.1;
            }
        }
    }
}
