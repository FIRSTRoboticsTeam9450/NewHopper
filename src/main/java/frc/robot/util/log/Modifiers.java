package frc.robot.util.log;

/**
 * Enum for java reflection modifier flags <br>
 * To be replaced by {@link java.lang.reflect.AccessFlag} in Java 21
 */
public enum Modifiers {

    PUBLIC(0x00000001),
    PRIVATE(0x00000002),
    PROTECTED(0x00000004),
    STATIC(0x00000008),
    FINAL(0x00000010),
    SYNCHRONIZED(0x00000020),
    VOLATILE(0x00000040),
    TRANSIENT(0x00000080),
    NATIVE(0x00000100),
    INTERFACE(0x00000200),
    ABSTRACT(0x00000400),
    STRICT(0x00000800),
    SYNTHETIC(0x00001000);

    final int mask;

    Modifiers(int mask) { this.mask = mask; }

    public int mask() { return mask; }

}
