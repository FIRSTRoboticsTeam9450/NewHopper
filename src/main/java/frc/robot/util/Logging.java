package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;

public class Logging {

    private final String PATH;
    private final List<Object> toLog;

    public Logging(String PATH) {

        this.PATH = PATH;
        toLog = new ArrayList<>();

    }

    public void add(Object object) {
        toLog.add(object);
    }

    public void run() {

        for(Object log : toLog) {
            log(log);
        }

    }

    private void log(Object o) {
        Class<?> clazz = o.getClass();
        Field[] fields = clazz.getDeclaredFields();
        for(Field field : fields) {
            if(field.getType().getPackage() == null) {
                field.setAccessible(true);
                try {
                    Object value = field.get(o);
                    String path = PATH + clazz.getPackageName().replace('.', '/') + "/" + clazz.getSimpleName() + "/" + field.getName();
                    Logger.recordOutput(path, (value == null) ? "null" : value.toString());
                } catch (IllegalAccessException e) {
                    System.out.println("Couldn't access field " + field.getName());
                }
            }
        }
    }

}
