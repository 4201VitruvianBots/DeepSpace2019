package frc.vitruvianlib.VitruvianLogger;


public class VitruvianLogField {
    public String name;
    public FunctionCall functionHandle;

    public VitruvianLogField(String name, FunctionCall functionHandle) {
        this.name = name;
        this.functionHandle = functionHandle;
    }

    public interface FunctionCall {
        Object get();
    }
}
