package frc.robot;

// TODO: verificar todos los IDs con electrónica antes de la primera prueba

/**
 * Centralización de IDs de hardware del robot.
 *
 * Separación de buses:
 *   - "canivore": bus CAN FD dedicado (CANivore USB-CAN bridge). Soporta 1 Mbit/s vs
 *     los 1 Mbit/s nominales del bus RoboRIO, pero con mayor determinismo y menor latencia.
 *     Se usa para dispositivos críticos de control de movimiento (swerve + Pigeon2) donde
 *     la latencia de odometría a 250 Hz importa.
 *   - "rio": bus CAN nativo del RoboRIO. Suficiente para mecanismos que no requieren
 *     odometría de alta frecuencia.

 */
public final class Ports {

    private Ports() {}

    // =========================================================================
    // CANivore — bus CAN FD dedicado, nombre de red: "canivore"
    // =========================================================================

    /** Nombre del bus CANivore tal como aparece en Phoenix Tuner X. */
    public static final String CANIVORE_BUS = "canivore";

    public static final class Swerve {

        // --- Módulo Frontal Izquierdo (FL) ---
        //Aztech
        /** TalonFX que controla la tracción FL; recibe VelocityVoltage/VoltageOut. */
        public static final int FL_DRIVE_ID = 1;

        /** TalonFX que controla el giro FL; cierra lazo sobre el CANcoder fusionado. */
        public static final int FL_STEER_ID = 2;

        /**
         * El CANcoder persiste la posición absoluta al encender → no se necesita
         * rutina de homing.
         */
        public static final int FL_CANCODER_ID = 3;

        // --- Módulo Frontal Derecho (FR) ---

        /** TalonFX tracción FR. */
        public static final int FR_DRIVE_ID = 4;

        /** TalonFX giro FR. */
        public static final int FR_STEER_ID = 5;

        /** CANcoder absoluto FR. */
        public static final int FR_CANCODER_ID = 6;

        // --- Módulo Trasero Izquierdo (BL) ---

        /** TalonFX tracción BL. */
        public static final int BL_DRIVE_ID = 7;

        /** TalonFX giro BL. */
        public static final int BL_STEER_ID = 8;

        /** CANcoder absoluto BL. */
        public static final int BL_CANCODER_ID = 9;

        // --- Módulo Trasero Derecho (BR) ---

        /** TalonFX tracción BR. */
        public static final int BR_DRIVE_ID = 10;

        /** TalonFX giro BR. */
        public static final int BR_STEER_ID = 11;

        /** CANcoder absoluto BR. */
        public static final int BR_CANCODER_ID = 12;
    }

    /**
     * IMU de 9 ejes. En FTC se usaría un BNO055/IMU interno del Control Hub;
     * el Pigeon2 vive en el CANivore para que Phoenix6 fusione su yaw directamente
     * en la odometría del swerve sin pasar por el RoboRIO.
     */
    public static final int PIGEON2_ID = 13;

    // =========================================================================
    // RoboRIO CAN — bus CAN nativo, nombre de red: "rio"
    // =========================================================================

    /** Nombre del bus RoboRIO; en Phoenix6 se pasa como string vacío o "rio". */
    public static final String RIO_BUS = "rio";

    public static final class Shooter {

        /**
         * TalonFX rueda disparadora izquierda. TalonFX ≈ Motor + SparkMax + encoder
         * todo en uno; la configuración se hace vía Configurator en código o Tuner X,
         */
        public static final int LEFT_ID = 20;

        /** TalonFX rueda disparadora derecha. Generalmente configurado como Follower. */
        public static final int RIGHT_ID = 21;
    }

    public static final class Hood {

        /**
         * TalonFX (Kraken X44 mini) lado izquierdo del capuchón (ajuste de ángulo
         * de disparo). Kraken mini = menor tamaño que Kraken X60 pero misma interfaz
         * Phoenix6; se configura idéntico al TalonFX estándar.
         */
        public static final int LEFT_ID = 22;

        /** TalonFX (Kraken mini) lado derecho del hood. */
        public static final int RIGHT_ID = 23;
    }

    /**
     * TalonFX del indexer/feeder: transporta la nota desde la intake hasta las
     * ruedas de disparo. Vive en el bus rio porque no requiere lazo de odometría.
     */
    public static final int INDEXER_ID = 24;

    public static final class Intake {

        /**
         * SPARK MAX izquierdo (ecosistema REV, protocolo CAN estándar 1 Mbit/s).
         * A diferencia de un TalonFX, el SPARK MAX se configura con SparkMaxConfig
         * y no es compatible con Phoenix Tuner X — usar REV Hardware Client para
         * diagnóstico y flasheo de firmware.
         */
        public static final int LEFT_SPARK_ID = 18;

        /** SPARK MAX derecho de la intake. */
        public static final int RIGHT_SPARK_ID = 15;

        public static final int ROLLER_ID = 28;

    }
}
