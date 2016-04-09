/*
        <Program Name>
            BackhaulService.java

        <Purpose>
            A background service that backhauls data sent to
            it by other activities to a remote server.
 */

package com.sensibilitytestbed.whereability;


import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;
import android.net.NetworkInfo;
import android.os.Binder;
import android.os.IBinder;

import org.json.JSONArray;
import org.json.JSONObject;

import java.io.BufferedOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;



public class BackhaulService extends Service {

    public static final int BATCH_SIZE = 100;

    private BlockingQueue queue = new LinkedBlockingQueue();
    private BackhaulBinder mBinder = new BackhaulBinder();
    private boolean started = false, done = false;
    private JSONArray batch = new JSONArray();
    private Lock lock = new ReentrantLock(), doneLock = new ReentrantLock();
    private Condition notFull = lock.newCondition(), notDone = doneLock.newCondition();
    private ConnectivityManager mConnectivityManager;
    private NetworkInfo mActiveNetwork;

    @Override
    public void onCreate() {

        mConnectivityManager = (ConnectivityManager) this.getSystemService(Context.CONNECTIVITY_SERVICE);
        mActiveNetwork = mConnectivityManager.getActiveNetworkInfo();

        started = true;

        new Thread(new Runnable() {
            public void run() {
                android.os.Process.setThreadPriority(android.os.Process.THREAD_PRIORITY_BACKGROUND);

                // Keep backhauling until told to stop
                // and the buffer is empty.
                while (started || !queue.isEmpty()) {

                    lock.lock();
                    try {
                        // Wait until there's a full batch or the
                        // service has been told to stop.
                        while (queue.size() < BATCH_SIZE && started)
                            notFull.await();
                    } catch (InterruptedException e) {

                    } finally {
                       lock.unlock();
                    }



                    // Consume the next batch of entries (i.e. measurements).
                    while (batch.length() < BATCH_SIZE && !queue.isEmpty())
                        try {
                            batch.put(queue.take());
                        } catch (InterruptedException e) { }



                    // Sleep until there's Internet connectivity.
                    while (mActiveNetwork == null || !mActiveNetwork.isConnectedOrConnecting()) {
                        if (mActiveNetwork == null)
                            mActiveNetwork = mConnectivityManager.getActiveNetworkInfo();
                        try {
                            Thread.sleep(5000);
                        } catch (InterruptedException e) { }
                    }



                    // Send the batch to the server
                    try {
                        // Connect to sensevis
                        String body = "user=Sensibility&exp=Indoor%20Localization&set=test&entries=" + batch.toString();
                        URL url = new URL("http://sensevis.poly.edu:8000/backhaul");
                        HttpURLConnection conn = (HttpURLConnection) url.openConnection();

                        // Declare message as POST
                        conn.setDoOutput(true);

                        // Declare length of packet body
                        conn.setFixedLengthStreamingMode(body.length());

                        // Send the data
                        OutputStream ostream = new BufferedOutputStream(conn.getOutputStream());
                        ostream.write(body.getBytes());
                        ostream.flush();

                        conn.disconnect();

                    } catch (IOException e) {
                        // Lost connection to the server.
                        // Try again in next go around.
                    }
                }


                // Finished backhauling, so signal
                // that it's okay to destroy the service.
                doneLock.lock();
                try {
                    done = true;
                    notDone.signal();
                } finally {
                    doneLock.unlock();
                }

            }
        }).start();
    }





    @Override
    public IBinder onBind(Intent intent) {
        return mBinder;
    }

    public class BackhaulBinder extends Binder {
        BackhaulService getService() {
            return BackhaulService.this;
        }
    }




    // Interface for other activities to backhaul data
    public void put(JSONObject entry) {
        try {
            queue.put(entry);
        } catch (InterruptedException e) {
          // Move on (toss entry) even if we failed to insert it,
          // so we don't keep blocking the sensor service.
        }

        lock.lock();
        try {
            // Signal consumer (network) thread
            // when to send another packet.
            if (queue.size() >= BATCH_SIZE)
                notFull.signal();
        } finally {
            lock.unlock();
        }
    }





    @Override
    public void onDestroy() {
        started = false;

        // Don't let the service die until the
        // buffer has been completely backhauled.
        doneLock.lock();
        try {
            if (!done)
                notDone.await();
        } catch (InterruptedException e) {

        } finally {
            doneLock.unlock();
        }
    }
}
