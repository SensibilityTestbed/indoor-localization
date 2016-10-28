/*
        <Program Name>
            BackhaulService.java

        <Purpose>
            A background service that manages multiple threads
            which backhaul data sent to this service by the
            MotionCaptureService. For every experiment (i.e.
            multiple experiments can be initiated via different
            QR codes), one thread is created which will backhaul
            sensor data to the experiment's storage path.
 */

package com.sensibilitytestbed.whereability;


import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.net.ConnectivityManager;

import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import org.json.JSONObject;


import java.util.HashMap;


import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;



public class BackhaulService extends Service {

    private BackhaulBinder mBinder = new BackhaulBinder();
    private ConnectivityManager connManager;
    private Lock lock = new ReentrantLock();
    private Lock doneLock = new ReentrantLock();
    private Condition notDone = doneLock.newCondition();
    private int numThreads = 0;
    private HashMap<String, Backhauler> backhaulers;


    @Override
    public void onCreate() {
        super.onCreate();
        connManager = (ConnectivityManager) this.getSystemService(Context.CONNECTIVITY_SERVICE);
        backhaulers = new HashMap();
    }


    @Override
    public void onDestroy() {
        stopBackhauling();
        Log.d("WhereAbility", "BackhaulService destroyed!");
        super.onDestroy();
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


    public void stopBackhauling() {
        Log.d("WhereAbility", "In stopBackhauling()");

        lock.lock();
        try {
            for (Backhauler backhauler: backhaulers.values())
                backhauler.kill();
        }
        finally {
            lock.unlock();
        }


        Log.d("WhereAbility", "Threads killed");
        // Don't let the service die until all the
        // backhauling threads have died.
        doneLock.lock();
        try {
            while(numThreads > 0)
                notDone.await();
        }
        catch(InterruptedException e) {
            Log.d("WhereAbility", "stopBackhauling()", e);
        }
        finally {
            doneLock.unlock();
        }

        Log.d("WhereAbility", "End of stopBackhauling()");
    }


    // Interface for other activities to backhaul data
    public void put(JSONObject entry) {
        lock.lock();
        try {
            for (Backhauler backhauler : backhaulers.values())
                backhauler.put(entry);
        }
        finally {
            lock.unlock();
        }
    }


    public void addPath(String path, boolean online) {

        /*
        Backhauler backhauler = getBackhauler(path);
        // Are we already writing to this path locally?
        if (!online && backhauler != null) {
            backhauler.kill();
            doneLock.lock();
            try {
                // Wait for the backhauler thread to die
                while(backhauler.isAlive())
                    notDone.await();
            } catch (InterruptedException e) {
                Log.d("WhereAbility", "addPath()", e);
            } finally {
                doneLock.unlock();
            }
        }
        */

        Backhauler backhauler;
        if(online)
            backhauler = new RemoteStore(path, this, connManager);
        else
            backhauler = new LocalStore(path, this);

        boolean inc = true;

        lock.lock();
        try {

            /*if(!backhaulers.containsKey(path)) {
                backhaulers.put(path, backhauler);
                */
            if(backhaulers.containsKey(path))
                inc = false;


            backhaulers.put(path, backhauler);
        }
        finally {
            lock.unlock();
        }
        new Thread(backhauler).start();

        if(inc) {
            doneLock.lock();
            numThreads++;
            doneLock.unlock();
        }
    }


    /*
    private Backhauler getBackhauler(String path) {
        lock.lock();
        try {
            if (backhaulers.containsKey(path))
                return backhaulers.get(path);
            else
                return null;
        }
        finally {
            lock.unlock();
        }
    }
    */



    public void signalDone() {
        Log.d("WhereAbility", "Thread done");
        doneLock.lock();
        try {
            numThreads--;
            // Kill the service once all
            // threads have finished
            if(numThreads == 0)
                notDone.signal();
        }
        finally {
            doneLock.unlock();
        }
    }

}