package com.team254.lib.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class CircularBufferTest {
    @Test
    void recomputeAveragePreservesAverageWhenBufferNotFull() {
        CircularBuffer buffer = new CircularBuffer(5);
        buffer.addValue(1.0);
        buffer.addValue(3.0);

        assertEquals(2.0, buffer.getAverage(), 1e-9);

        buffer.recomputeAverage();

        assertEquals(2.0, buffer.getAverage(), 1e-9);
    }

    @Test
    void recomputeAveragePreservesAverageWhenBufferFull() {
        CircularBuffer buffer = new CircularBuffer(3);
        buffer.addValue(1.0);
        buffer.addValue(2.0);
        buffer.addValue(3.0);

        assertEquals(2.0, buffer.getAverage(), 1e-9);

        buffer.recomputeAverage();

        assertEquals(2.0, buffer.getAverage(), 1e-9);
    }
}
