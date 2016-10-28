/*
        <Program Name>
            RemoteStore.java

        <Author>
            Seth Miller

        <Purpose>
            A child of the Backhauler class responsible for backhauling data to
            a remote server.
 */

package com.sensibilitytestbed.whereability;


import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.util.Log;

import org.json.JSONArray;

import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;


public class RemoteStore extends Backhauler {
    private ConnectivityManager connManager;

    public RemoteStore(String path, BackhaulService service, ConnectivityManager cm) {
        super(path, service);
        connManager = cm;
    }

    public void run() {

        HttpURLConnection conn = null;
        URL url = null;
        try {
            url = new URL(path);
            conn = (HttpURLConnection) url.openConnection();
        } catch (Exception e) {
            Log.e("WhereAbility", "exception", e);
        }

        NetworkInfo activeNetwork = connManager.getActiveNetworkInfo();

        while(!dead || !queue.isEmpty()) {

            while (activeNetwork == null || !activeNetwork.isConnectedOrConnecting()) {
                if (activeNetwork == null)
                    activeNetwork = connManager.getActiveNetworkInfo();
                try {
                    Thread.sleep(5000);
                }
                catch(InterruptedException e) {
                    Log.e("WhereAbility", "exception", e);
                }
            }

            if (conn == null) {
                try {
                    conn = (HttpURLConnection) url.openConnection();
                }
                catch(Exception e) {
                    continue;
                }
            }
            try {
                int code = conn.getResponseCode();
                if (code != 200)
                    conn.connect();
            }
            catch(IOException e) {
                Log.d("WhereAbility", "RemoteStore", e);
                continue;
            }

            JSONArray batch = getBatch();

            // Send the batch to the server
            try {
                // Declare message as POST
                conn.setDoOutput(true);

                String body = "";
                try {
                    body = batch.toString();
                }
                catch(NullPointerException e) {
                    Log.d("WhereAbility", "RemoteStore", e);
                }

                // Declare length of packet body
                conn.setFixedLengthStreamingMode(body.length());

                // Send the data
                OutputStream ostream = new BufferedOutputStream(conn.getOutputStream());
                ostream.write(body.getBytes());
                ostream.flush();

                // Did the message go through?
                if (conn.getResponseCode() != 200)
                    replaceBatch(batch);

            }
            catch(IOException e) {
                // Lost connection to the server.
                // Try again in next go around.
                replaceBatch(batch);
            }
        }
        try {
            conn.disconnect();
        }
        catch(Exception e) {
            Log.d("WhereAbility", "RemoteStore", e);
        }
        service.signalDone();
    }
}
