## UML Diagram
```
classDiagram
    class max30102_data {
        +struct i2c_client *client
        +rwlock_t lock
        +struct work_struct work
        +struct gpio_desc *irq_gpio
        +struct gpio_desc *reset_gpio
        +struct miscdevice miscdev
        +struct input_dev *input_dev
        +struct regulator *vcc_regulator
        +struct device *hwmon_dev
        +struct max30102_fifo fifo
        +uint32_t red_data[32]
        +uint32_t ir_data[32]
        +uint8_t data_len
        +bool fifo_full
        +wait_queue_head_t wait_data_ready
        +struct dentry *debug_dir
        +struct rt_mutex fifo_mutex
        +struct srcu_struct fifo_srcu
        +atomic_t futex_val
        +atomic_t temp_ready
        +u64 rcu_gp_seq
    }

    class max30102_fifo {
        +struct max30102_fifo_entry entries[32]
        +atomic_t head
        +atomic_t tail
    }

    class max30102_fifo_entry {
        +uint32_t red
        +uint32_t ir
        +struct rcu_head rcu
    }

    class max30102_core {
        +max30102_probe()
        +max30102_remove()
        +max30102_suspend()
        +max30102_resume()
        +temperature_show()
        +status_show()
        +led_current_show()
        +led_current_store()
    }

    class max30102_i2c {
        +max30102_write_reg()
        +max30102_read_reg()
    }

    class max30102_interrupt {
        +max30102_irq_handler()
        +max30102_work_handler()
    }

    class max30102_data_ops {
        +max30102_read_fifo()
        +max30102_read_temperature()
        +max30102_rcu_free()
    }

    class max30102_config {
        +max30102_init_sensor()
        +max30102_set_mode()
        +max30102_set_slot()
        +max30102_set_interrupt()
        +max30102_set_fifo_config()
        +max30102_set_spo2_config()
    }

    class max30102_ioctl {
        +max30102_open()
        +max30102_ioctl()
        +max30102_compat_ioctl()
    }

    class max30102_user {
        +main()
        +fifo_thread()
        +temp_thread()
        +signal_handler()
        +futex_waitv()
    }

    class max30102_ebpf {
        +bpf_probe_rt_mutex_lock()
        +bpf_retprobe_rt_mutex_lock()
        +bpf_probe_spin_lock()
        +bpf_retprobe_spin_lock()
        +bpf_probe_i2c_transfer()
        +bpf_retprobe_i2c_transfer()
    }

    max30102_data o--> max30102_fifo : contains
    max30102_fifo o--> max30102_fifo_entry : contains
    max30102_core --> max30102_data : uses
    max30102_i2c --> max30102_data : uses
    max30102_interrupt --> max30102_data : uses
    max30102_data_ops --> max30102_data : uses
    max30102_config --> max30102_data : uses
    max30102_ioctl --> max30102_data : uses
    max30102_user --> max30102_ioctl : calls
    max30102_ebpf --> max30102_data : monitors
```

<img width="1563" height="944" alt="image" src="https://github.com/user-attachments/assets/1718464b-ccac-4e73-ae0b-bba4b105eb0e" />


## Architecture Flow
```
graph TD
    A[MAX30102 Sensor] -->|I2C Data| B[I2C Interface: max30102_i2c.c]
    A -->|Interrupt| C[IRQ Handler: max30102_interrupt.c]
    
    subgraph Kernel Space
        B -->|Read/Write Registers| D[Core Driver: max30102_core.c]
        C -->|Schedule Work| E[Workqueue: max30102_work_handler]
        E -->|FIFO Write| F[Lock-Free FIFO: max30102_data.c]
        F -->|SRCU Read| G[Data Processing: max30102_data.c]
        G -->|Heart Rate/SpO2| H[Input Subsystem]
        G -->|Temperature| I[HWMON Subsystem]
        D -->|Sysfs Attributes| J[Sysfs: /sys/bus/i2c/devices]
        D -->|Debugfs| K[Debugfs: /sys/kernel/debug/max30102]
        D -->|IOCTL Interface| L[IOCTL: max30102_ioctl.c]
        L -->|PI Mutex| M[FIFO Mutex with Timeout]
        F -->|Futex Wake| N[Futex: atomic_t futex_val, temp_ready]
        O[eBPF Monitoring: max30102_ebpf.c] -->|Trace Locks/I2C| P[/sys/kernel/debug/tracing]
    end
    
    subgraph User Space
        Q[User App: max30102_user.c] -->|IOCTL Calls| L
        Q -->|Futex Wait Multiple| N
        Q -->|SCHED_DEADLINE| R[Real-Time Threads]
        Q -->|Read FIFO/Temp| S[/dev/max30102]
        Q -->|Shared Memory| T[/dev/shm/max30102_shm]
        Q -->|Message Queue| U[/max30102_mq]
        Q -->|FIFO| V[/tmp/max30102_fifo]
        W[Benchmark Script: max30102_benchmark.sh] -->|Perf/Taskset| Q
        W -->|eBPF Trace| P
    end
    
    subgraph Device Tree
        X[max30102.dts] -->|Overlay| D
    end
```

```mermaid
graph TD
    A[MAX30102 Sensor] -->|I2C Data| B[I2C Interface\nmax30102_i2c.c]
    A -->|Interrupt Trigger| C[IRQ Handler\nmax30102_interrupt.c]
    
    subgraph Kernel Space
        B -->|Register Read/Write| D[Core Driver\nmax30102_core.c]
        C -->|Schedule Workqueue| E[Workqueue\nmax30102_work_handler]
        E -->|Write to FIFO| F[Lock-Free FIFO\nmax30102_data.c]
        F -->|SRCU Read| G[Data Processing\nmax30102_data.c]
        G -->|Report HR/SpO2| H[Input Subsystem]
        G -->|Report Temperature| I[HWMON Subsystem]
        D -->|Expose Attributes| J[Sysfs\n/sys/bus/i2c/devices]
        D -->|Expose Debug Info| K[Debugfs\n/sys/kernel/debug/max30102]
        D -->|Provide IOCTL| L[IOCTL Interface\nmax30102_ioctl.c]
        L -->|Protect with PI Mutex| M[FIFO Mutex\nwith Timeout]
        F -->|Signal via Futex| N[Futex\natomic_t futex_val, temp_ready]
        O[eBPF Monitoring\nmax30102_ebpf.c] -->|Trace Locks/I2C| P[Tracing\n/sys/kernel/debug/tracing]
    end
    
    subgraph User Space
        Q[User App\nmax30102_user.c] -->|Call IOCTL| L
        Q -->|Futex Wait Multiple| N
        Q -->|Run Real-Time Threads| R[SCHED_DEADLINE\nThreads]
        Q -->|Read FIFO/Temp| S[Device\n/dev/max30102]
        Q -->|Access Shared Memory| T[Shared Memory\n/dev/shm/max30102_shm]
        Q -->|Access Message Queue| U[Message Queue\n/max30102_mq]
        Q -->|Access FIFO| V[FIFO\n/tmp/max30102_fifo]
        W[Benchmark Script\nmax30102_benchmark.sh] -->|Run Perf/Taskset| Q
        W -->|Analyze eBPF Trace| P
    end
    
    subgraph Device Tree
        X[max30102.dts] -->|Apply Overlay| D
    end

    classDef kernel fill:#f9f,stroke:#333,stroke-width:2px;
    classDef user fill:#bbf,stroke:#333,stroke-width:2px;
    classDef hardware fill:#bfb,stroke:#333,stroke-width:2px;
    classDef device_tree fill:#ffb,stroke:#333,stroke-width:2px;

    class A hardware;
    class B,C,D,E,F,G,H,I,J,K,L,M,N,O,P kernel;
    class Q,R,S,T,U,V,W user;
    class X device_tree;
```
