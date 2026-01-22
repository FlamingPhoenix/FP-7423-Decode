package org.firstinspires.ftc.teamcode.utility;
import android.content.Context;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;

public class PersistentStorage {

    private static final String FILE_NAME = "config.txt";

    public static void saveX(Context context, int x) {
        try (FileOutputStream fos = context.openFileOutput(FILE_NAME, Context.MODE_PRIVATE)) {
            String data = String.valueOf(x);
            fos.write(data.getBytes());
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static int loadX(Context context, int defaultValue) {
        try (FileInputStream fis = context.openFileInput(FILE_NAME)) {
            byte[] bytes = new byte[fis.available()];
            fis.read(bytes);
            String data = new String(bytes);
            return Integer.parseInt(data.trim());
        } catch (Exception e) {
            // If file not found or parse error, return default value
            return defaultValue;
        }
    }
}
