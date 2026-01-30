package org.firstinspires.ftc.teamcode.utility;

import android.content.Context;

import java.io.BufferedReader;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

public class PersistentStorage {

    private static final String FILE_NAME = "config.txt";

    // Save one double value under a key
    public static boolean saveDouble(Context context, String key, double value) {
        try {
            Map<String, String> data = loadAll(context);
            data.put(key, String.valueOf(value));
            saveAll(context, data);
            return true;
        } catch (Exception e) {
            e.printStackTrace();
            return false; // FAIL CASE
        }
    }

    // Load one double value from a key
    public static double loadDouble(Context context, String key, double defaultValue) {
        try {
            Map<String, String> data = loadAll(context);

            if (!data.containsKey(key)) {
                return defaultValue; // FAIL CASE: key missing
            }

            String raw = data.get(key);
            if (raw == null) return defaultValue; // fail case: key exists but value is null

            return Double.parseDouble(raw.trim());
        } catch (Exception e) {
            return defaultValue; // FAIL CASE: file missing / parse error / etc.
        }
    }

    // ======== Internal Helpers ========

    private static Map<String, String> loadAll(Context context) {
        Map<String, String> map = new HashMap<>();

        try (FileInputStream fis = context.openFileInput(FILE_NAME);
             BufferedReader br = new BufferedReader(new InputStreamReader(fis))) {

            String line;
            while ((line = br.readLine()) != null) {
                line = line.trim();
                if (line.isEmpty()) continue;

                int eq = line.indexOf('=');
                if (eq <= 0) continue;

                String key = line.substring(0, eq).trim();
                String value = line.substring(eq + 1).trim();
                map.put(key, value);
            }

        } catch (Exception e) {
            // FAIL CASE: file doesn't exist yet -> return empty map
        }

        return map;
    }

    private static void saveAll(Context context, Map<String, String> map) throws IOException {
        StringBuilder sb = new StringBuilder();

        for (Map.Entry<String, String> entry : map.entrySet()) {
            sb.append(entry.getKey())
                    .append("=")
                    .append(entry.getValue())
                    .append("\n");
        }

        try (FileOutputStream fos = context.openFileOutput(FILE_NAME, Context.MODE_PRIVATE)) {
            fos.write(sb.toString().getBytes());
        }
    }
}
