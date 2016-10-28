/*
        <Program Name>
            LocalStore.java

        <Author>
            Seth Miller

        <Purpose>
            A child of the Backhauler class responsible for storing data locally
            (i.e. to a file on the SD card).
 */


package com.sensibilitytestbed.whereability;

import android.util.Log;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileWriter;
import java.io.IOException;

public class LocalStore extends Backhauler {


    public LocalStore(String path, BackhaulService service) {
        super(path, service);
    }

    public void run() {
        FileWriter fileWriter = null;
        try {
            fileWriter = new FileWriter(path);
            fileWriter.write("time,deviceID,height,location,ax,ay,az,gx,gy,gz,mx,my,mz\n");
        }
        catch (IOException e) {
            Log.d("WhereAbility", "LocalStore", e);
        }


        while(isAlive() || !queue.isEmpty()) {

            Log.d("WhereAbility", "Going around the loop");

            JSONArray batch = getBatch();

            // Write JSON objects in array to a file
            for(int i = 0; i < batch.length(); i++) {
                JSONObject row;
                try {
                    row = (JSONObject) batch.get(i);
                }
                catch(JSONException e) {
                    Log.d("WhereAbility", "LocalStore", e);
                    continue;
                }
                catch(NullPointerException e) {
                    break; // empty batch
                }

                // Convert JSON objects to CSV rows
                String csv = "";
                for(String key : KEYS) {
                    try {
                        csv += "" + row.get(key) + ",";
                    }
                    catch(JSONException e) {
                        // skip missing sensors
                        csv += ",";
                    }
                }
                csv += "\n";

                try {
                    fileWriter.write(csv);
                }
                catch(Exception e) {
                    Log.d("WhereAbility", "LocalStore", e);
                    replaceBatch(batch);
                    break;
                }
            }
        }

        Log.d("WhereAbility", "Out of the loop");

        try {
            fileWriter.close();
        }
        catch(Exception e) {
            Log.d("WhereAbility", "LocalStore", e);
        }
        service.signalDone();
    }
}
