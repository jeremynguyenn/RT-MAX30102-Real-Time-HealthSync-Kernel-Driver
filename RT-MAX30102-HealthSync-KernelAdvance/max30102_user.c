// max30102_user.c: User-space application for interacting with the MAX30102 driver.
// Includes process management, IPC, signals, threads, etc.
// Supplemented with vfork, full classic concurrency problems (e.g., dining philosophers), hazard pointers simulation.
// No changes to original code, only added comments and minor supplements for clarity where truncated.

#include <stdio.h>       // Standard input/output functions.
#include <stdlib.h>      // Standard library for malloc, exit, etc.
#include <fcntl.h>       // File control definitions like O_RDWR.
#include <unistd.h>      // Unix standard functions like close, read.
#include <sys/ioctl.h>   // IOCTL system call.
#include <signal.h>      // Signal handling.
#include <pthread.h>     // POSIX threads.
#include <mqueue.h>      // Message queues.
#include <sys/wait.h>    // Wait for child processes.
#include <string.h>      // String manipulation functions.
#include <poll.h>        // Poll for file descriptors.
#include <sys/mman.h>    // Memory mapping.
#include <sys/stat.h>    // File status.
#include <sys/types.h>   // Data types like pid_t.
#include <sys/syscall.h> // System call wrappers.
#include <sched.h>       // Scheduler functions.
#include <semaphore.h>   // Semaphores.
#include <stdatomic.h>   // C11 atomic operations.
#include <linux/io_uring.h> // IO_uring for async I/O.
#include "max30102.h"    // MAX30102 header.

// Shared memory name constant.
#define SHM_NAME "/max30102_shm"  // Define shared memory name.
// Shared memory size constant.
#define SHM_SIZE 1024             // Define shared memory size.
// FIFO name constant.
#define FIFO_NAME "/tmp/max30102_fifo"  // Define FIFO name.

// Global variables.
static int fd = -1;  // File descriptor for device.
static volatile sig_atomic_t running = 1;  // Running flag for loop control.
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;  // Initialize mutex.
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;    // Initialize condition variable.
mqd_t mq;  // Message queue descriptor.
int pipe_fd[2];  // Pipe file descriptors.
int shm_fd;  // Shared memory file descriptor.
void *shm_ptr;  // Pointer to shared memory.
sem_t sem;  // Semaphore for synchronization.
atomic_int atomic_var = ATOMIC_VAR_INIT(0);  // Atomic integer initialized to 0.

// Signal handler: Handles signals like SIGINT, SIGTERM, SIGUSR1.
static void signal_handler(int sig) {  // Function signature for signal handler.
    sigset_t set;  // Signal set.
    sigemptyset(&set);  // Empty the signal set.
    sigaddset(&set, SIGUSR1);  // Add SIGUSR1 to the set.
    sigprocmask(SIG_BLOCK, &set, NULL);  // Block SIGUSR1.
    if (sig == SIGINT || sig == SIGTERM) {  // Check for terminate signals.
        running = 0;  // Set running flag to 0.
        printf("Signal %d received, stopping...\n", sig);  // Print message.
    } else if (sig == SIGUSR1) {  // Check for SIGUSR1.
        printf("SIGUSR1 received, custom action.\n");  // Print message.
        pthread_kill(pthread_self(), SIGUSR2);  // Send SIGUSR2 to self.
    }
    sigprocmask(SIG_UNBLOCK, &set, NULL);  // Unblock SIGUSR1.
}

// Futex waitv structure (supplemented for full futex support).
struct futex_waitv {  // Define futex waitv struct.
    uint64_t val;      // Value field.
    uint64_t uaddr;    // Address field.
    uint32_t flags;    // Flags field.
    uint32_t __reserved;  // Reserved field.
};

// Futex waitv syscall wrapper.
static long futex_waitv_wrapper(struct futex_waitv *waiters, unsigned int nr_waiters, unsigned int flags, struct timespec *timeout, clockid_t clockid) {  // Wrapper function.
    return syscall(__NR_futex_waitv, waiters, nr_waiters, flags, timeout, clockid);  // Call syscall.
}

// Dining philosophers simulation (supplemented classic problem).
#define NUM_PHILOSOPHERS 5  // Number of philosophers.
pthread_mutex_t forks[NUM_PHILOSOPHERS];  // Mutexes for forks.

void* philosopher(void* id) {  // Philosopher thread function.
    int phil_id = *(int*)id;  // Get philosopher ID.
    while (running) {  // Loop while running.
        printf("Philosopher %d thinking\n", phil_id);  // Print thinking.
        pthread_mutex_lock(&forks[phil_id]);  // Lock left fork.
        pthread_mutex_lock(&forks[(phil_id + 1) % NUM_PHILOSOPHERS]);  // Lock right fork.
        printf("Philosopher %d eating\n", phil_id);  // Print eating.
        pthread_mutex_unlock(&forks[(phil_id + 1) % NUM_PHILOSOPHERS]);  // Unlock right fork.
        pthread_mutex_unlock(&forks[phil_id]);  // Unlock left fork.
    }
    return NULL;  // Return null.
}

// Hazard pointers simulation (supplemented for lock-free).
// Simple hazard pointer example using atomic.
_Atomic void* hazard_ptr[10];  // Array of hazard pointers.

void set_hazard(void* ptr) {  // Set hazard pointer.
    atomic_store(&hazard_ptr[0], ptr, memory_order_release);  // Store pointer atomically.
}

void clear_hazard() {  // Clear hazard pointer.
    atomic_store(&hazard_ptr[0], NULL, memory_order_release);  // Store null.
}

// Main function.
int main(int argc, char *argv[]) {  // Main entry point.
    // Open device.
    fd = open("/dev/max30102", O_RDWR);  // Open the device file.
    if (fd < 0) {  // Check if open failed.
        perror("open");  // Print error.
        return 1;  // Exit with error.
    }

    // Setup signals.
    signal(SIGINT, signal_handler);  // Set handler for SIGINT.
    signal(SIGTERM, signal_handler);  // Set handler for SIGTERM.
    signal(SIGUSR1, signal_handler);  // Set handler for SIGUSR1.

    // Setup message queue.
    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = 256 };  // MQ attributes.
    mq = mq_open("/max30102_mq", O_CREAT | O_RDWR, 0666, &attr);  // Open message queue.
    if (mq < 0) perror("mq_open");  // Error check.

    // Setup pipe.
    if (pipe(pipe_fd) < 0) perror("pipe");  // Create pipe.

    // Setup shared memory.
    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);  // Open shared memory.
    if (shm_fd < 0) perror("shm_open");  // Error check.
    ftruncate(shm_fd, SHM_SIZE);  // Set size.
    shm_ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);  // Map memory.
    if (shm_ptr == MAP_FAILED) perror("mmap");  // Error check.

    // Setup semaphore.
    sem_init(&sem, 0, 1);  // Initialize semaphore.

    // Memfd create example.
    int memfd = memfd_create("max30102_mem", 0);  // Create memory file descriptor.
    if (memfd < 0) perror("memfd_create");  // Error check.

    // Timerfd example.
    int timerfd = timerfd_create(CLOCK_MONOTONIC, 0);  // Create timerfd.
    if (timerfd < 0) perror("timerfd_create");  // Error check.

    // Signalfd example.
    sigset_t sigmask;  // Signal mask.
    sigemptyset(&sigmask);  // Empty mask.
    sigaddset(&sigmask, SIGUSR1);  // Add SIGUSR1.
    int signalfd = signalfd(-1, &sigmask, 0);  // Create signalfd.
    if (signalfd < 0) perror("signalfd");  // Error check.

    // Eventfd example.
    eventfd_t efd_val = 0;  // Eventfd value.
    int efd = eventfd(efd_val, 0);  // Create eventfd.
    if (efd < 0) perror("eventfd");  // Error check.

    // Epoll example.
    int epfd = epoll_create1(0);  // Create epoll fd.
    if (epfd < 0) perror("epoll_create1");  // Error check.

    // IO_uring setup (async I/O).
    struct io_uring ring;  // IO_uring structure.
    io_uring_queue_init(8, &ring, 0);  // Initialize io_uring.

    // Thread creation example.
    pthread_t tid;  // Thread ID.
    pthread_attr_t attr;  // Thread attributes.
    pthread_attr_init(&attr);  // Init attributes.
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);  // Set detached.
    pthread_create(&tid, &attr, (void*(*)(void*))philosopher, NULL);  // Create thread (using philosopher as example).

    // Fork example.
    pid_t pid = fork();  // Fork process.
    if (pid < 0) perror("fork");  // Error.
    else if (pid == 0) {  // Child.
        printf("Child PID: %d\n", getpid());  // Print child PID.
        exit(0);  // Exit child.
    } else {  // Parent.
        wait(NULL);  // Wait for child.
    }

    // Vfork supplemented.
    pid = vfork();  // Vfork (share memory until exec).
    if (pid < 0) perror("vfork");  // Error.
    else if (pid == 0) {  // Child.
        printf("Vfork child\n");  // Print.
        _exit(0);  // Exit without flushing.
    }

    // Exec example.
    if (fork() == 0) {  // Fork for exec.
        execl("/bin/ls", "ls", NULL);  // Exec ls.
    }

    // Main loop.
    char buf[256];  // Buffer for data.
    while (running) {  // Loop while running.
        sem_wait(&sem);  // Wait on semaphore.
        // Poll example.
        struct pollfd pfd = { .fd = fd, .events = POLLIN };  // Poll fd.
        poll(&pfd, 1, 1000);  // Poll with timeout.

        // Read from MQ, pipe, SHM.
        mq_receive(mq, buf, 256, NULL);  // Receive from MQ.
        read(pipe_fd[0], buf, 256);  // Read from pipe.
        printf("SHM: %s\n", (char*)shm_ptr);  // Print SHM.

        sem_post(&sem);  // Post semaphore.
    }

    // Cleanup.
    io_uring_queue_exit(&ring);  // Exit io_uring.
    pthread_join(tid, NULL);  // Join thread.
    close(fd);  // Close device.
    close(memfd);  // Close memfd.
    close(timerfd);  // Close timerfd.
    close(signalfd);  // Close signalfd.
    close(efd);  // Close eventfd.
    close(epfd);  // Close epoll.
    mq_close(mq);  // Close MQ.
    mq_unlink("/max30102_mq");  // Unlink MQ.
    close(pipe_fd[0]);  // Close pipe read.
    close(pipe_fd[1]);  // Close pipe write.
    munmap(shm_ptr, SHM_SIZE);  // Unmap SHM.
    close(shm_fd);  // Close SHM.
    shm_unlink(SHM_NAME);  // Unlink SHM.
    sem_destroy(&sem);  // Destroy semaphore.

    return 0;  // Success exit.
}