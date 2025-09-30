#!/bin/bash

# Kiểm tra môi trường
if ! command -v perf >/dev/null 2>&1; then
    echo "Error: perf not installed. Please install linux-tools-common."
    exit 1
fi

if ! command -v taskset >/dev/null 2>&1; then
    echo "Error: taskset not installed. Please install util-linux."
    exit 1
fi

# Benchmark futex latency
echo "Benchmarking futex latency..."
perf record -e futex:* -o futex_perf.data ./max30102_user &
PID=$!
sleep 10
kill $PID
wait $PID 2>/dev/null
perf report -i futex_perf.data > futex_report.txt
echo "Futex report saved to futex_report.txt"

# Benchmark poll latency
echo "Benchmarking poll latency..."
perf record -e syscalls:sys_enter_poll,syscalls:sys_exit_poll -o poll_perf.data ./max30102_user &
PID=$!
sleep 10
kill $PID
wait $PID 2>/dev/null
perf report -i poll_perf.data > poll_report.txt
echo "Poll report saved to poll_report.txt"

# Benchmark concurrency with taskset
echo "Benchmarking concurrency on CPUs 0,1..."
taskset -c 0,1 perf stat -e cache-misses,cycles,instructions -o concurrency_report.txt ./max30102_user &
PID=$!
sleep 10
kill $PID
wait $PID 2>/dev/null
echo "Concurrency report saved to concurrency_report.txt"

# Benchmark I2C contention with eBPF
echo "Monitoring I2C contention with eBPF..."
sudo bpftrace -e '
    kprobe:i2c_transfer {
        @start[tid] = nsecs;
    }
    kretprobe:i2c_transfer {
        if (@start[tid]) {
            @delta[tid] = (nsecs - @start[tid]) / 1000;
            if (@delta[tid] > 500) {
                printf("I2C contention: TID %d took %lld us\n", tid, @delta[tid]);
            }
            delete(@start[tid]);
            delete(@delta[tid]);
        }
    }
' > ebpf_contention.txt &
BPF_PID=$!
sleep 10
sudo kill $BPF_PID
wait $BPF_PID 2>/dev/null
echo "eBPF contention report saved to ebpf_contention.txt"

# Cleanup
rm -f futex_perf.data poll_perf.data
echo "Benchmark completed. Check reports: futex_report.txt, poll_report.txt, concurrency_report.txt, ebpf_contention.txt"