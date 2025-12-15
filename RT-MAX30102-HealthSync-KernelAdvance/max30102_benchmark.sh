#!/bin/bash  # Shebang for bash.

# max30102_benchmark.sh: Benchmark script for the driver.
# Supplemented with additional benchmarks for concurrency.
# No changes to original code, only added comments.

# Check perf.
if ! command -v perf >/dev/null 2>&1; then  # Check if perf installed.
    echo "Error: perf not installed. Please install linux-tools-common."  # Error.
    exit 1  # Exit.
fi

# Check taskset.
if ! command -v taskset >/dev/null 2>&1; then  # Check taskset.
    echo "Error: taskset not installed. Please install util-linux."  # Error.
    exit 1  # Exit.
fi

# Benchmark futex.
echo "Benchmarking futex latency..."  # Message.
perf record -e futex:* -o futex_perf.data ./max30102_user &  # Record futex.
PID=$!  # Get PID.
sleep 10  # Sleep.
kill $PID  # Kill.
wait $PID 2>/dev/null  # Wait.
perf report -i futex_perf.data > futex_report.txt  # Report.
echo "Futex report saved to futex_report.txt"  # Message.

# Benchmark poll.
echo "Benchmarking poll latency..."  # Message.
perf record -e syscalls:sys_enter_poll,syscalls:sys_exit_poll -o poll_perf.data ./max30102_user &  # Record poll.
PID=$!  # PID.
sleep 10  # Sleep.
kill $PID  # Kill.
wait $PID 2>/dev/null  # Wait.
perf report -i poll_perf.data > poll_report.txt  # Report.
echo "Poll report saved to poll_report.txt"  # Message.

# Benchmark concurrency.
echo "Benchmarking concurrency on CPUs 0,1..."  # Message.
taskset -c 0,1 perf stat -e cache-misses,cycles,instructions -o concurrency_report.txt ./max30102_user &  # Stat with taskset.
PID=$!  # PID.
sleep 10  # Sleep.
kill $PID  # Kill.
wait $PID 2>/dev/null  # Wait.
echo "Concurrency report saved to concurrency_report.txt"  # Message.

# eBPF for I2C contention.
echo "Monitoring I2C contention with eBPF..."  # Message.
sudo bpftrace -e '  # bpftrace script.
    kprobe:i2c_transfer {
        @start[tid] = nsecs;  # Start timestamp.
    }
    kretprobe:i2c_transfer {
        if (@start[tid]) {  # If start exists.
            @delta[tid] = (nsecs - @start[tid]) / 1000;  # Calculate delta in us.
            if (@delta[tid] > 500) {  # If >500us.
                printf("I2C contention: TID %d took %lld us\n", tid, @delta[tid]);  # Print.
            }
            delete(@start[tid]);  # Delete start.
            delete(@delta[tid]);  # Delete delta.
        }
    }
' > ebpf_contention.txt &  # Output to file.
BPF_PID=$!  # PID.
sleep 10  # Sleep.
sudo kill $BPF_PID  # Kill.
wait $BPF_PID 2>/dev/null  # Wait.
echo "eBPF contention report saved to ebpf_contention.txt"  # Message.

# Cleanup.
rm -f futex_perf.data poll_perf.data  # Remove data files.
echo "Benchmark completed. Check reports: futex_report.txt, poll_report.txt, concurrency_report.txt, ebpf_contention.txt"  # Final message.