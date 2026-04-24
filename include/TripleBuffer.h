#pragma once
#include <atomic>
#include <memory>

/**
 * @brief Thread-safe Triple Buffer for single-producer, single-consumer.
 * Using atomic pointer exchange to ensure lock-free synchronization.
 */
template <typename T>
class TripleBuffer {
public:
    TripleBuffer(const T& initialValue = T()) {
        // Allocate 3 buffers
        buffers[0] = new T(initialValue);
        buffers[1] = new T(initialValue);
        buffers[2] = new T(initialValue);

        // Assign initial roles
        m_writerPtr = buffers[0];
        m_sharedPtr.store(buffers[1]);
        m_readerPtr = buffers[2];
    }

    ~TripleBuffer() {
        delete buffers[0];
        delete buffers[1];
        delete buffers[2];
    }

    /**
     * @brief [Writer Thread] Update the state.
     * Writes to the private back buffer, then publishes it.
     */
    void write(const T& value) {
        // 1. Write to private back buffer
        *m_writerPtr = value;

        // 2. Publish: Swap Writer Ptr with Shared Ptr
        // The old Shared (which might be what Reader just finished with, 
        // or an intermediate frame) becomes our new Back buffer to overwrite next.
        m_writerPtr = m_sharedPtr.exchange(m_writerPtr);
    }

    /**
     * @brief [Reader Thread] Get the latest state.
     * Swaps the private front buffer with the shared buffer.
     * @return The latest available data.
     */
    T read() {
        // 1. Acquire: Swap Reader Ptr with Shared Ptr
        // We give the Shared slot our old data (to be recycled by writer eventually)
        // and take whatever is currently in Shared (the freshest committed write).
        m_readerPtr = m_sharedPtr.exchange(m_readerPtr);
        
        // 2. Return data
        return *m_readerPtr;
    }

    /**
     * @brief [Reader Thread] Peek without swapping (not thread safe if writer is active? No, just unsafe).
     * Actually 'read()' is cheap enough, just use that. 
     * But if we want to avoid modifying state just to peek? 
     * Standard usage should be: render loop calls read() once per frame.
     */

private:
    T* buffers[3]; 
    
    T* m_writerPtr;               // Owned by Writer thread (Private)
    std::atomic<T*> m_sharedPtr;  // The "Fresh" buffer (Shared)
    T* m_readerPtr;               // Owned by Reader thread (Private)
};
