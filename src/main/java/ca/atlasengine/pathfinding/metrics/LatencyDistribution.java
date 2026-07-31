package ca.atlasengine.pathfinding.metrics;

import java.util.ArrayList;
import java.util.List;

/**
 * Immutable bucketed view of one latency series, in microseconds.
 *
 * <p>Bucket {@code 0} counts sub-microsecond samples; bucket {@code i} counts
 * samples in {@code [2^(i-1), 2^i)} microseconds; the final bucket is unbounded
 * above. Quantiles are therefore exact to a factor of two and always round up,
 * so a reported tail is never optimistic.</p>
 */
public record LatencyDistribution(
        long count, long totalMicros, long maximumMicros,
        List<Long> bucketCounts) {

    public static final int BUCKETS = 32;

    public LatencyDistribution {
        if (bucketCounts.size() != BUCKETS) {
            throw new IllegalArgumentException("bucketCounts");
        }
        bucketCounts = List.copyOf(bucketCounts);
    }

    public static LatencyDistribution empty() {
        return of(new long[BUCKETS], 0, 0);
    }

    static LatencyDistribution of(
            long[] counts, long totalMicros, long maximumMicros) {
        List<Long> boxed = new ArrayList<>(BUCKETS);
        long count = 0;
        for (long bucket : counts) {
            boxed.add(bucket);
            count += bucket;
        }
        return new LatencyDistribution(
                count, totalMicros, maximumMicros, boxed);
    }

    static int bucketOf(long micros) {
        if (micros <= 0) return 0;
        return Math.min(BUCKETS - 1, 64 - Long.numberOfLeadingZeros(micros));
    }

    /** Inclusive upper bound of one bucket; the final bucket is unbounded. */
    public static long bucketUpperBoundMicros(int index) {
        if (index <= 0) return 0;
        if (index >= BUCKETS - 1) return Long.MAX_VALUE;
        return (1L << index) - 1;
    }

    public double meanMicros() {
        return count == 0 ? 0 : (double) totalMicros / count;
    }

    /**
     * Upper bound of the bucket holding the requested rank, clamped by the
     * largest observed sample. Never reports below the true quantile.
     */
    public long percentileMicros(double quantile) {
        if (!(quantile >= 0) || quantile > 1) {
            throw new IllegalArgumentException("quantile");
        }
        if (count == 0) return 0;
        long rank = Math.max(1, (long) Math.ceil(quantile * count));
        long seen = 0;
        for (int index = 0; index < BUCKETS; index++) {
            seen += bucketCounts.get(index);
            if (seen >= rank) {
                return Math.min(bucketUpperBoundMicros(index), maximumMicros);
            }
        }
        return maximumMicros;
    }

    public long p50Micros() {
        return percentileMicros(0.50);
    }

    public long p90Micros() {
        return percentileMicros(0.90);
    }

    public long p99Micros() {
        return percentileMicros(0.99);
    }

    /**
     * Distribution of only the samples recorded after {@code earlier}, for
     * windowed tail reporting between two snapshots of the same recorder. The
     * maximum of a window is bucket-resolution rather than exact.
     */
    public LatencyDistribution since(LatencyDistribution earlier) {
        if (earlier == null) throw new IllegalArgumentException("earlier");
        long[] delta = new long[BUCKETS];
        long highest = 0;
        for (int index = 0; index < BUCKETS; index++) {
            delta[index] = Math.max(0,
                    bucketCounts.get(index) - earlier.bucketCounts.get(index));
            if (delta[index] > 0) {
                highest = Math.min(maximumMicros,
                        bucketUpperBoundMicros(index));
            }
        }
        return of(delta, Math.max(0, totalMicros - earlier.totalMicros),
                highest);
    }
}
