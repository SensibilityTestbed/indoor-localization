/*
        <Program Name>
            Backhauler.java

        <Author>
            Seth Miller

        <Purpose>
            An abstract Runnable (thread) object that provides the basic
            functionality for backhauling sensor data and interfacing with
            the BackhaulService managing it.
 */

package com.sensibilitytestbed.whereability;

import android.util.Log;

import org.json.JSONArray;
import org.json.JSONObject;

import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;


public abstract class Backhauler implements Runnable{
    private static final int BUFFER_MAX = 600000;
    protected static final int BATCH_SIZE = 500;
    protected static final String[] KEYS = {
            "time", "deviceID",
            "height", "location",
            "ax", "ay", "az",
            "gx", "gy", "gz",
            "mx", "my", "mz"
    };

    protected String path;
    protected BackhaulService service;
    protected boolean dead;
    protected BlockingQueue queue;
    protected ReentrantLock lock;
    protected Condition notFull;

    public Backhauler(String path, BackhaulService s) {
        this.path = path;
        service = s;
        dead = false;
        queue = new LinkedBlockingQueue();
        lock = new ReentrantLock();
        notFull = lock.newCondition();
    }


    public void kill() {
        lock.lock();
        dead = true;
        notFull.signal();
        lock.unlock();
    }


    public void put(JSONObject entry) {
        try {
            if (queue.size() > BUFFER_MAX)
                return;
            queue.put(entry);
        } catch (InterruptedException e) {
            // Move on (i.e. toss entry) even if we failed to insert it,
            // so we don't keep blocking the sensor service.
        } catch (NullPointerException e) {
            return;
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



    protected boolean isAlive() {
        lock.lock();
        try {
            return !dead;
        }
        finally {
            lock.unlock();
        }
    }




    protected JSONArray getBatch() {
        Log.d("WhereAbility", "In getBatch()");
        // Wait until there's a full batch
        // or we've been told to stop.
        lock.lock();
        try {
            while (queue.size() < BATCH_SIZE && isAlive())
                notFull.await();
        } catch (InterruptedException e) {
            Log.d("WhereAbility", "getBatch()", e);
        }
        finally {
            lock.unlock();
        }

        Log.d("WhereAbility", "Done waiting for a batch");

        JSONArray batch = new JSONArray();
        // Consume the next batch of entries (i.e. measurements).
        while(batch.length() < BATCH_SIZE && !queue.isEmpty()) {
            try {
                batch.put(queue.take());
            }
            catch(InterruptedException e) {
                Log.d("WhereAbility", "getBatch()", e);
            }
        }
        Log.d("WhereAbility", "End of getBatch()");
        return batch;
    }




    protected void replaceBatch(JSONArray array) {
        try {
            for(int i = 0; i < array.length(); i++)
                queue.put(array.get(i));
        }
        catch(Exception e) {
            Log.d("WhereAbility", "replaceBatch()", e);
        }
    }
}
