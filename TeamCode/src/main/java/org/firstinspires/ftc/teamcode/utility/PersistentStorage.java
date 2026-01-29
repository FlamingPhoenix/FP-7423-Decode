package org.firstinspires.ftc.teamcode.utility;
import android.content.Context;

import java.io.ByteArrayOutputStream;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

public class PersistentStorage {

    private static final String FILE_NAME = "config.txt";

    public static void saveX(Context context, int x) {
        try (FileOutputStream fos = context.openFileOutput(FILE_NAME, Context.MODE_PRIVATE)) {
            String data = String.valueOf(x);
            fos.write(data.getBytes());
        }
        catch (IOException e) {
            e.printStackTrace();
//            throw new RuntimeException("Failed to save data", e);
        }
    }

    public static int loadX(Context context, int defaultValue) {
        try (FileInputStream fis = context.openFileInput(FILE_NAME)) {
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            byte[] buffer = new byte[64];
            int n;
            while ((n = fis.read(buffer)) != -1) {
                baos.write(buffer, 0, n);
            }
            return Integer.parseInt(baos.toString().trim());
        } catch (Exception e) {
            return defaultValue;
        }
    }
}
