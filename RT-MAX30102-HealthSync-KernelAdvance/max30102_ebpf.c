// max30102_ebpf.c: eBPF program for tracing locks and I2C.
// Supplemented with KCSAN-like tracing.
// No changes to original code, only added comments.

#include <linux/bpf.h>         // BPF.
#include <bpf/bpf_helpers.h>   // Helpers.
#include <bpf/bpf_tracing.h>   // Tracing.

// Map for lock times.
struct {  // Map definition.
    __uint(type, BPF_MAP_TYPE_HASH);  // Hash type.
    __uint(max_entries, 1024);  // Max entries.
    __type(key, u32);  // Key PID.
    __type(value, u64);  // Value timestamp.
} lock_map SEC(".maps");  // Section maps.

SEC("kprobe/rt_mutex_lock")  // Section for probe.
int bpf_probe_rt_mutex_lock(struct pt_regs *ctx) {  // Probe function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // Get PID.
    u64 ts = bpf_ktime_get_ns();  // Get timestamp.
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);  // Update map.
    bpf_trace_printk("RT lock PID %d CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);  // Trace.
    return 0;  // Success.
}

SEC("kretprobe/rt_mutex_lock")  // Return probe.
int bpf_retprobe_rt_mutex_lock(struct pt_regs *ctx) {  // Function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // PID.
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);  // Lookup.
    if (ts) {  // If found.
        u64 delta = bpf_ktime_get_ns() - *ts;  // Delta.
        if (delta > 1000000000) bpf_trace_printk("Livelock: PID %d held %llu ns\n", pid, delta);  // Trace if long.
        bpf_map_delete_elem(&lock_map, &pid);  // Delete.
    }
    return 0;  // Success.
}

SEC("kprobe/_raw_spin_lock_irqsave")  // Probe for spinlock.
int bpf_probe_spin_lock(struct pt_regs *ctx) {  // Function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // PID.
    u64 ts = bpf_ktime_get_ns();  // TS.
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);  // Update.
    bpf_trace_printk("Spinlock PID %d CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);  // Trace.
    return 0;  // Success.
}

SEC("kretprobe/_raw_spin_lock_irqsave")  // Return probe.
int bpf_retprobe_spin_lock(struct pt_regs *ctx) {  // Function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // PID.
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);  // Lookup.
    if (ts) {  // Found.
        u64 delta = bpf_ktime_get_ns() - *ts;  // Delta.
        if (delta > 1000000000) bpf_trace_printk("Livelock spin: PID %d held %llu ns\n", pid, delta);  // Trace.
        bpf_map_delete_elem(&lock_map, &pid);  // Delete.
    }
    return 0;  // Success.
}

SEC("kprobe/i2c_transfer")  // Probe for I2C.
int bpf_probe_i2c_transfer(struct pt_regs *ctx) {  // Function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // PID.
    u64 ts = bpf_ktime_get_ns();  // TS.
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);  // Update.
    bpf_trace_printk("I2C PID %d CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);  // Trace.
    return 0;  // Success.
}

SEC("kretprobe/i2c_transfer")  // Return probe.
int bpf_retprobe_i2c_transfer(struct pt_regs *ctx) {  // Function.
    u32 pid = bpf_get_current_pid_tgid() >> 32;  // PID.
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);  // Lookup.
    if (ts) {  // Found.
        u64 delta = bpf_ktime_get_ns() - *ts;  // Delta.
        if (delta > 500000000) bpf_trace_printk("I2C contention: PID %d %llu ns\n", pid, delta);  // Trace.
        bpf_map_delete_elem(&lock_map, &pid);  // Delete.
    }
    return 0;  // Success.
}

char _license[] SEC("license") = "GPL";  // License.
u32 _version SEC("version") = LINUX_VERSION_CODE;  // Version.
// Note: Use KCSAN for data races in kernel compile.