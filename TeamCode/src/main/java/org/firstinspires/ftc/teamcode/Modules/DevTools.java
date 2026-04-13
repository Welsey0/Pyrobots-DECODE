package org.firstinspires.ftc.teamcode.Modules;

import java.util.List;
import java.util.ArrayList;
import java.util.Collections;
import org.firstinspires.ftc.teamcode.MDS;

public class DevTools {
    @SuppressWarnings({"FieldCanBeLocal", "unused"})
    private MDS mds;
    public void setMDS(MDS mds) {
        this.mds = mds;
    }

    // fps tracking variables
    public long fps = 0;
    public long frametime = 0;
    private long startTime = System.nanoTime();

    // adv metrics
    @SuppressWarnings("FieldMayBeFinal")
    private List<Long> frametimeHistory = new ArrayList<>();
    public long average = 0;
    public long low = 0;

    public void updatefps() {
        long newTime = System.nanoTime();
        long deltaTime = (newTime - startTime) / 1_000_000; // convert to milliseconds

        if (deltaTime > 0) {
            frametime = deltaTime;
            fps = 1000 / frametime; // fps calc

            // store ft for adv metrics
            frametimeHistory.add(frametime);
            if (frametimeHistory.size() > 1000) {
                frametimeHistory.remove(0);
            }

            // calculate adv metrics every so often
            if (frametimeHistory.size() >= 100) {
                calcAdvMetrics();
            }
        }
        startTime = newTime;
    }

    private void calcAdvMetrics() {
        // average fps
        long totalFrametime = 0;
        for (Long time : frametimeHistory) {
            totalFrametime += time;
        }
        long averageFrametime = totalFrametime / frametimeHistory.size();
        average = averageFrametime > 0 ? 1000 / averageFrametime : 0;

        // 1% low fps
        List<Long> sortedFrametimes = new ArrayList<>(frametimeHistory);
        Collections.sort(sortedFrametimes);

        // Calculate the index for 1% low
        int onePercentIndex = (int)(sortedFrametimes.size() * 0.01);
        if (onePercentIndex > 0) {
            long onePercentFrametime = sortedFrametimes.get(onePercentIndex);
            low = onePercentFrametime > 0 ? 1000 / onePercentFrametime : 0;
        }
    }

    @SuppressWarnings("unused")
    public void resetFpsHistory() {
        frametimeHistory.clear();
        average = 0;
        low = 0;
    }
}
