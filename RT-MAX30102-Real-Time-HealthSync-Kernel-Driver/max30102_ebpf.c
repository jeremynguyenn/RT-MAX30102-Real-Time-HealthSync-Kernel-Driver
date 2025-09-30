#include <linux/bpf.h>
#include <bpf/bpf_helpers.h>
#include <bpf/bpf_tracing.h>

struct {
    __uint(type, BPF_MAP_TYPE_HASH);
    __uint(max_entries, 1024);
    __type(key, u32);
    __type(value, u64);
} lock_map SEC(".maps");

SEC("kprobe/rt_mutex_lock")
int bpf_probe_rt_mutex_lock(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 ts = bpf_ktime_get_ns();
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);
    bpf_trace_printk("RT mutex lock by PID %d on CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);
    return 0;
}

SEC("kretprobe/rt_mutex_lock")
int bpf_retprobe_rt_mutex_lock(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);
    if (ts) {
        u64 delta = bpf_ktime_get_ns() - *ts;
        if (delta > 1000000000) bpf_trace_printk("Potential livelock/deadlock: PID %d held RT mutex for %llu ns\n", pid, delta);
        bpf_map_delete_elem(&lock_map, &pid);
    }
    return 0;
}

SEC("kprobe/_raw_spin_lock_irqsave")
int bpf_probe_spin_lock(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 ts = bpf_ktime_get_ns();
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);
    bpf_trace_printk("Spinlock taken by PID %d on CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);
    return 0;
}

SEC("kretprobe/_raw_spin_lock_irqsave")
int bpf_retprobe_spin_lock(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);
    if (ts) {
        u64 delta = bpf_ktime_get_ns() - *ts;
        if (delta > 1000000000) bpf_trace_printk("Potential livelock: PID %d held spinlock for %llu ns\n", pid, delta);
        bpf_map_delete_elem(&lock_map, &pid);
    }
    return 0;
}

SEC("kprobe/i2c_transfer")
int bpf_probe_i2c_transfer(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 ts = bpf_ktime_get_ns();
    bpf_map_update_elem(&lock_map, &pid, &ts, BPF_ANY);
    bpf_trace_printk("I2C transfer by PID %d on CPU %d at %llu\n", pid, bpf_get_smp_processor_id(), ts);
    return 0;
}

SEC("kretprobe/i2c_transfer")
int bpf_retprobe_i2c_transfer(struct pt_regs *ctx)
{
    u32 pid = bpf_get_current_pid_tgid() >> 32;
    u64 *ts = bpf_map_lookup_elem(&lock_map, &pid);
    if (ts) {
        u64 delta = bpf_ktime_get_ns() - *ts;
        if (delta > 500000000) bpf_trace_printk("I2C contention: PID %d took %llu ns\n", pid, delta);
        bpf_map_delete_elem(&lock_map, &pid);
    }
    return 0;
}

char _license[] SEC("license") = "GPL";
u32 _version SEC("version") = LINUX_VERSION_CODE;