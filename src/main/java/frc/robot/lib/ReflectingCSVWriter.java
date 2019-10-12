package frc.robot.lib;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.lang.reflect.Field;
import java.util.concurrent.ConcurrentLinkedDeque;

public class ReflectingCSVWriter<T> {
    ConcurrentLinkedDeque<String> mLinesToWrite = new ConcurrentLinkedDeque<>();
    PrintWriter mOutput = null;
    Field[] mFields;

    public ReflectingCSVWriter(String fileName, Class<T> typeClass) {
        mFields = typeClass.getFields();
        try {
            mOutput = new PrintWriter(fileName);
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        }

        StringBuffer line = new StringBuffer();
        for (Field mField : mFields){
            if (line.length() != 0){
                line.append(", ");
            }
            line.append(mField.getName());
        }
        writeLine(line.toString());
    }

    public void add(T val){
        StringBuffer line = new StringBuffer();
        for (Field mField : mFields){
            if (line.length() != 0) {
                line.append(", ");
            }
            try {
                line.append(mField.get(val).toString());
            } catch (IllegalArgumentException e) {
                e.printStackTrace();
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }
        mLinesToWrite.add(line.toString());
    }

    protected synchronized void writeLine(String line){
        if (mOutput != null) {
            mOutput.println(line);
        }
    }

    public void write(){
        while (true) {
            String sVal = mLinesToWrite.pollFirst();
            if (sVal == null){
                break;
            }
            writeLine(sVal);
        }
    }

    public synchronized void flush() {
        if (mOutput != null){
            write();
            mOutput.flush();
        }
    }
}