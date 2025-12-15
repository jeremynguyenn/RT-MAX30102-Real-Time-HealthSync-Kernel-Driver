// max30102_user_advanced.c: Advanced user-space concurrency examples.
// Includes recursive mutex, PI mutex, deadlock prevention, readers-writers, spinlock, barrier.
// Supplemented with deadline scheduling mention, deterministic timing comment.
// No changes to original code, only added comments and minor supplements for clarity where truncated.

#include <pthread.h>    // POSIX threads.
#include <stdatomic.h>  // Atomics.
#include <semaphore.h>  // Semaphores.
#include <stdio.h>      // I/O.

// Recursive mutex.
pthread_mutex_t recursive_mutex;  // Recursive mutex.

void init_recursive_mutex() {  // Init function.
    pthread_mutexattr_t attr;  // Attributes.
    pthread_mutexattr_init(&attr);  // Init attr.
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);  // Set recursive type.
    pthread_mutex_init(&recursive_mutex, &attr);  // Init mutex.
    pthread_mutexattr_destroy(&attr);  // Destroy attr.
}

void use_recursive() {  // Use function.
    pthread_mutex_lock(&recursive_mutex);  // Lock.
    pthread_mutex_lock(&recursive_mutex);  // Nested lock.
    printf("Recursive lock\n");  // Print.
    pthread_mutex_unlock(&recursive_mutex);  // Unlock nested.
    pthread_mutex_unlock(&recursive_mutex);  // Unlock.
}

// PI mutex.
pthread_mutex_t pi_mutex;  // PI mutex.

void init_pi_mutex() {  // Init function.
    pthread_mutexattr_t attr;  // Attributes.
    pthread_mutexattr_init(&attr);  // Init.
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);  // Set priority inheritance.
    pthread_mutex_init(&pi_mutex, &attr);  // Init mutex.
    pthread_mutexattr_destroy(&attr);  // Destroy.
}

void use_pi() {  // Use function.
    pthread_mutex_lock(&pi_mutex);  // Lock with PI.
    printf("PI mutex lock\n");  // Print.
    pthread_mutex_unlock(&pi_mutex);  // Unlock.
}

// Deadlock prevention.
int try_lock_to_prevent_deadlock(pthread_mutex_t *m1, pthread_mutex_t *m2) {  // Function to prevent deadlock.
    if (pthread_mutex_trylock(m1) == 0) {  // Try lock m1.
        if (pthread_mutex_trylock(m2) == 0) {  // Try lock m2.
            pthread_mutex_unlock(m2);  // Unlock m2.
            pthread_mutex_unlock(m1);  // Unlock m1.
            return 0;  // Success.
        } else {
            pthread_mutex_unlock(m1);  // Unlock m1 if m2 fails.
        }
    }
    return -1;  // Failed.
}

// Readers-writers.
pthread_rwlock_t rwlock;  // Read-write lock.

void init_rwlock() {  // Init function.
    pthread_rwlock_init(&rwlock, NULL);  // Init rwlock.
}

void reader() {  // Reader function.
    pthread_rwlock_rdlock(&rwlock);  // Read lock.
    printf("Reading\n");  // Print.
    pthread_rwlock_unlock(&rwlock);  // Unlock.
}

void writer() {  // Writer function.
    pthread_rwlock_wrlock(&rwlock);  // Write lock.
    printf("Writing\n");  // Print.
    pthread_rwlock_unlock(&rwlock);  // Unlock.
}

// User spinlock.
_Atomic int user_spin = 0;  // Atomic spin variable.

void user_spin_lock() {  // Spin lock function.
    while (atomic_exchange(&user_spin, 1, memory_order_acquire));  // Spin until acquire.
}

void user_spin_unlock() {  // Unlock function.
    atomic_store(&user_spin, 0, memory_order_release);  // Release.
}

// Barrier.
pthread_barrier_t barrier;  // Barrier.

void init_barrier(int count) {  // Init function.
    pthread_barrier_init(&barrier, NULL, count);  // Init for count threads.
}

void use_barrier() {  // Use function.
    pthread_barrier_wait(&barrier);  // Wait at barrier.
}

// Note: For deadline scheduling, use SCHED_DEADLINE (real-time).
// Deterministic timing: Use high-priority threads with fixed periods.