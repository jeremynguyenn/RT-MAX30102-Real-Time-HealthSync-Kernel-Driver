#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <pthread.h>
#include <mqueue.h>
#include <sys/wait.h>
#include <string.h>
#include <poll.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/syscall.h>
#include <sched.h>
#include "max30102.h"

#define SHM_NAME "/max30102_shm"
#define SHM_SIZE 1024
#define FIFO_NAME "/tmp/max30102_fifo"

static int fd = -1;
static volatile sig_atomic_t running = 1;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t cond = PTHREAD_COND_INITIALIZER;
mqd_t mq;
int pipe_fd[2];
int shm_fd;
void *shm_ptr;

// Signal handler
static void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        running = 0;
        printf("Signal %d received, stopping...\n", sig);
    } else if (sig == SIGUSR1) {
        printf("SIGUSR1 received, custom action.\n");
    }
}

// Futex wait multiple
struct futex_waitv {
    uint64_t val;
    uint64_t uaddr;
    uint32_t flags;
    uint32_t __reserved;
};

static long futex_waitv(struct futex_waitv *waiters, unsigned int nr_futexes, unsigned int flags, struct timespec *timeout) {
    return syscall(SYS_futex_waitv, waiters, nr_futexes, flags, timeout, 0);
}

// FIFO thread (joinable, SCHED_DEADLINE)
void *fifo_thread(void *arg) {
    struct max30102_fifo_data fifo_data;
    char buf[256];
    struct pollfd pfd = { .fd = fd, .events = POLLIN };
    struct max30102_data *data = arg;
    pthread_t tid = pthread_self();
    printf("FIFO thread ID: %lu\n", (unsigned long)tid);

    // Set SCHED_DEADLINE
    struct sched_attr {
        size_t size;
        unsigned int sched_policy;
        unsigned long long sched_flags;
        int sched_nice;
        unsigned int sched_priority;
        unsigned long long sched_runtime;
        unsigned long long sched_deadline;
        unsigned long long sched_period;
    } attr = {
        .size = sizeof(attr),
        .sched_policy = SCHED_DEADLINE,
        .sched_runtime = 1000000,  // 1ms
        .sched_deadline = 2000000, // 2ms
        .sched_period = 2000000,
    };
    if (sched_setattr(0, &attr, 0) < 0) {
        perror("sched_setattr");
    }

    struct futex_waitv waitv[2] = {
        { .uaddr = (uint64_t)&data->futex_val, .flags = FUTEX_32 },
        { .uaddr = (uint64_t)&data->temp_ready, .flags = FUTEX_32 }
    };

    while (running) {
        int ret = futex_waitv(waitv, 2, 0, NULL);
        if (ret < 0) {
            perror("futex_waitv");
            break;
        }
        ret = pthread_mutex_lock(&mutex);
        if (ret != 0) perror("Mutex lock failed"), break;
        ret = ioctl(fd, MAX30102_IOC_READ_FIFO, &fifo_data);
        if (ret < 0) perror("Failed to read FIFO data"), pthread_mutex_unlock(&mutex), break;
        sprintf(buf, "FIFO: %d samples", fifo_data.len);
        ret = mq_send(mq, buf, strlen(buf) + 1, 0);
        if (ret < 0) perror("mq_send failed");
        ret = write(pipe_fd[1], buf, strlen(buf) + 1);
        if (ret < 0) perror("Pipe write failed");
        memcpy(shm_ptr, buf, strlen(buf) + 1);
        pthread_cond_signal(&cond);
        pthread_mutex_unlock(&mutex);
    }
    return NULL;
}

// Temp thread (detached)
void *temp_thread(void *arg) {
    float temp;
    static int static_var = 0;
    int auto_var = 0;
    pthread_t tid = pthread_self();
    printf("Temp thread ID: %lu\n", (unsigned long)tid);

    while (running) {
        int ret = pthread_mutex_lock(&mutex);
        if (ret != 0) perror("Mutex lock failed"), break;
        ret = ioctl(fd, MAX30102_IOC_READ_TEMP, &temp);
        if (ret < 0) perror("Failed to read temperature"), pthread_mutex_unlock(&mutex), break;
        auto_var++;
        static_var++;
        printf("Temp: %.4f°C, Auto: %d, Static: %d\n", temp, auto_var, static_var);
        pthread_mutex_unlock(&mutex);
        sleep(5);
    }
    return NULL;
}

int main(int argc, char *argv[]) {
    if (argc > 1) printf("Command line arg: %s\n", argv[1]);

    fd = open("/dev/max30102", O_RDWR | O_NONBLOCK);
    if (fd < 0) perror("Failed to open device"), return 1;

    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    if (sigaction(SIGINT, &sa, NULL) < 0 || sigaction(SIGTERM, &sa, NULL) < 0 || sigaction(SIGUSR1, &sa, NULL) < 0) {
        perror("sigaction");
        close(fd);
        return 1;
    }

    struct mq_attr attr = { .mq_maxmsg = 10, .mq_msgsize = 256 };
    mq = mq_open("/max30102_mq", O_CREAT | O_RDWR, 0666, &attr);
    if (mq == (mqd_t)-1) perror("mq_open"), close(fd), return 1;

    if (pipe(pipe_fd) < 0) perror("pipe"), mq_close(mq), mq_unlink("/max30102_mq"), close(fd), return 1;

    mkfifo(FIFO_NAME, 0666);
    int fifo_fd = open(FIFO_NAME, O_WRONLY | O_NONBLOCK);
    if (fifo_fd < 0) perror("open FIFO");

    shm_fd = shm_open(SHM_NAME, O_CREAT | O_RDWR, 0666);
    if (shm_fd < 0) perror("shm_open");
    else {
        if (ftruncate(shm_fd, SHM_SIZE) < 0) perror("ftruncate");
        shm_ptr = mmap(0, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd, 0);
        if (shm_ptr == MAP_FAILED) perror("mmap"), shm_ptr = NULL;
    }

    uint8_t mode = MAX30102_MODE_SPO2;
    struct max30102_slot_config slot_config = { .slot = 1, .led = 2 };
    uint8_t fifo_config = MAX30102_FIFO_SMP_AVE_8;
    uint8_t spo2_config = MAX30102_SPO2_CONFIG_DEFAULT;

    if (ioctl(fd, MAX30102_IOC_SET_FIFO_CONFIG, &fifo_config) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_SPO2_CONFIG, &spo2_config) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_MODE, &mode) < 0 ||
        ioctl(fd, MAX30102_IOC_SET_SLOT, &slot_config) < 0) {
        perror("Config ioctl failed");
        goto cleanup;
    }

    pthread_t fifo_tid, temp_tid;
    pthread_attr_t attr_detach;
    pthread_attr_init(&attr_detach);
    pthread_attr_setdetachstate(&attr_detach, PTHREAD_CREATE_DETACHED);

    int ret = pthread_create(&fifo_tid, NULL, fifo_thread, NULL);
    if (ret != 0) perror("pthread_create fifo"), goto cleanup;
    ret = pthread_create(&temp_tid, &attr_detach, temp_thread, NULL);
    if (ret != 0) perror("pthread_create temp"), goto cleanup;

    pid_t pid = fork();
    if (pid < 0) perror("fork"), goto cleanup;
    else if (pid == 0) {
        printf("Child PID: %d, Parent PID: %d\n", getpid(), getppid());
        char *args[] = {"echo", "Hello from exec", argv[1] ? argv[1] : "default", NULL};
        execvp("echo", args);
        perror("execvp");
        exit(1);
    } else {
        printf("Parent PID: %d waiting for child %d\n", getpid(), pid);
        int status;
        if (wait(&status) < 0) perror("wait");
        if (WIFEXITED(status)) printf("Child exited with status %d\n", WEXITSTATUS(status));
    }

    char buf[256];
    while (running) {
        int ret = pthread_mutex_lock(&mutex);
        if (ret != 0) perror("Mutex lock failed"), break;
        ret = pthread_cond_wait(&cond, &mutex);
        if (ret != 0) perror("Cond wait failed"), pthread_mutex_unlock(&mutex), break;
        ssize_t mq_ret = mq_receive(mq, buf, 256, NULL);
        if (mq_ret < 0) perror("mq_receive");
        else printf("Received from queue: %s\n", buf);
        ssize_t pipe_ret = read(pipe_fd[0], buf, 256);
        if (pipe_ret < 0) perror("pipe read");
        else printf("Received from pipe: %s\n", buf);
        if (shm_ptr) printf("Received from shm: %s\n", (char *)shm_ptr);
        pthread_mutex_unlock(&mutex);
    }

cleanup:
    pthread_join(fifo_tid, NULL);
    if (fifo_fd > 0) close(fifo_fd);
    unlink(FIFO_NAME);
    if (shm_ptr) munmap(shm_ptr, SHM_SIZE);
    if (shm_fd > 0) close(shm_fd);
    shm_unlink(SHM_NAME);
    close(pipe_fd[0]);
    close(pipe_fd[1]);
    mq_close(mq);
    mq_unlink("/max30102_mq");
    close(fd);
    return 0;
}