package frc.robot.util.log;

import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class Logging {

    public final String PATH;
    private final List<Object> toLog;
    private final HashMap<Class<?>, Handler> handlers;
    private int mask;
    private final LogLevel level;

    public Logging(String PATH, LogLevel level, Modifiers... exclude) {

        this.PATH = PATH;
        this.level = level;
        toLog = new ArrayList<>();
        handlers = new HashMap<>();

        for(Modifiers flag : exclude) {
            mask |= flag.mask();
        }

    }

    public void addLoggedObject(Object object) {
        toLog.add(object);
    }

    public void addHandler(Handler handler) {
        handlers.put(handler.clazz, handler);
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
            if((field.getModifiers() & mask) != 0) continue;
            if (level != LogLevel.FULL_DEBUG) {
                ToLog annotation = field.getAnnotation(ToLog.class);
                if (annotation == null) {
                    continue;
                } else {
                    if(annotation.level().compareTo(level) > 0) continue;
                }
            }
            try {
                Handler h;
                field.setAccessible(true);
                Object value = field.get(o);
                String path = PATH + clazz.getPackageName().replace('.', '/') + "/" + clazz.getSimpleName() + "/" + field.getName();
                if ((h = handlers.get(field.getType())) != null) {
                    h.handle(path, value);
                } else if (field.getType().getPackage() == null) {
                    logPrimitive(path, value);
                }
            } catch (IllegalAccessException e){
                System.out.println("Couldn't access field " + field.getName());
            }
        }
    }

    private void logPrimitive(String path, Object o) {
        if(o instanceof Number) {
            Logger.recordOutput(path, ((Number) o).doubleValue());
        } else if(o instanceof Character) {
            Logger.recordOutput(path, (Character) o);
        }else if(o instanceof Boolean) {
            Logger.recordOutput(path, (Boolean) o);
        }
    }

    public abstract static class Handler {

        public final Class<?> clazz;

        public Handler(Class<?> clazz) {
            this.clazz = clazz;
        }

        public abstract void handle(String path, Object o);

    }

}
