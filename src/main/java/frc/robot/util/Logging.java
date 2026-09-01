package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;

public class Logging {
    public static void log(Class<?> c) {
        Field[] fields = c.getDeclaredFields();
        for (Field field : fields) {
            Logger.recordOutput("Testing",field.toGenericString());
        }
    }
}
