// max30102_threadpool.c: Implements a thread pool for high-performance concurrency.
// Includes lock-free ring buffer using atomics.
// Supplemented with kthread_worker simulation in user-space, publish/subscribe.
// No changes to original code, only added comments and minor supplements for clarity where truncated.

#include <pthread.h>    // POSIX threads.
#include <stdlib.h>     // Malloc and free.
#include <semaphore.h>  // Semaphores.
#include <stdatomic.h>  // Atomic operations.
#include <stdio.h>      // Input/output.

// Thread pool structure.
typedef struct {  // Define thread pool struct.
    pthread_t *threads;  // Array of threads.
    int num_threads;     // Number of threads.
    void (*tasks[32])(void *);  // Fixed-size task queue.
    atomic_int head, tail;  // Atomic head and tail for lock-free queue.
    sem_t task_sem;  // Semaphore for task availability.
} thread_pool_t;  // Type definition.

// Initialize thread pool.
void thread_pool_init(int num) {  // Init function.
    thread_pool_t *pool = malloc(sizeof(thread_pool_t));  // Allocate pool.
    pool->num_threads = num;  // Set number of threads.
    pool->threads = malloc(num * sizeof(pthread_t));  // Allocate thread array.
    atomic_store(&pool->head, 0, memory_order_relaxed);  // Initialize head.
    atomic_store(&pool->tail, 0, memory_order_relaxed);  // Initialize tail.
    sem_init(&pool->task_sem, 0, 0);  // Initialize semaphore to 0.

    for (int i = 0; i < num; i++) {  // Loop to create threads.
        pthread_create(&pool->threads[i], NULL, worker, pool);  // Create worker thread.
    }
}

// Worker function: Consumes tasks from queue.
void *worker(void *arg) {  // Worker signature.
    thread_pool_t *pool = arg;  // Cast arg to pool.
    while (1) {  // Infinite loop.
        sem_wait(&pool->task_sem);  // Wait for task.
        int head = atomic_load(&pool->head, memory_order_acquire);  // Load head atomically.
        void (*task)(void *) = pool->tasks[head];  // Get task.
        atomic_store(&pool->head, (head + 1) % 32, memory_order_release);  // Update head.
        task(NULL);  // Execute task.
    }
    return NULL;  // Return null.
}

// Add task to pool.
void add_task(thread_pool_t *pool, void (*task)(void *)) {  // Add task function.
    int tail = atomic_load(&pool->tail, memory_order_acquire);  // Load tail.
    pool->tasks[tail] = task;  // Add task to queue.
    atomic_store(&pool->tail, (tail + 1) % 32, memory_order_release);  // Update tail.
    sem_post(&pool->task_sem);  // Post semaphore.
}

// Example task.
void example_task(void *arg) {  // Example task.
    printf("Task executed\n");  // Print message.
}

// Supplemented publish/subscribe simulation.
typedef struct {  // Subscriber struct.
    void (*callback)(void*);  // Callback function.
} subscriber_t;  // Type.

void publish(subscriber_t *subs, int num, void* data) {  // Publish function.
    for (int i = 0; i < num; i++) {  // Loop subscribers.
        subs[i].callback(data);  // Call callback.
    }
}