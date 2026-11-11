#pragma once
// Internal fork-join thread pool used by World (not part of the public API).

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <memory>
#include <thread>
#include <type_traits>
#include <vector>

#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__)
#include <immintrin.h>
#define ACCEL_CPU_RELAX() _mm_pause()
#else
#define ACCEL_CPU_RELAX() std::this_thread::yield()
#endif

namespace AccelEngine
{
    class ThreadPool
    {
    public:
        // `workers` threads are started; the calling thread also takes part in every parallelFor.
        explicit ThreadPool(int workers)
        {
            for (int i = 0; i < workers; i++)
                threads.emplace_back([this] { workerLoop(); });
        }

        ~ThreadPool()
        {
            quit.store(true, std::memory_order_release);
            generation.fetch_add(1, std::memory_order_acq_rel);
            generation.notify_all();
            for (auto &t : threads)
                t.join();
        }

        ThreadPool(const ThreadPool &) = delete;
        ThreadPool &operator=(const ThreadPool &) = delete;

        int threadCount() const { return (int)threads.size() + 1; }

        // Calls fn(begin, end, chunkIndex) for consecutive chunks of [0, count) of size `grain`.
        // Chunk c always covers [c*grain, ...), so per-chunk outputs can be concatenated in order for
        // deterministic results. Small jobs run inline. Must be called from one thread at a time.
        template <typename F>
        void parallelFor(int count, int grain, F &&fn)
        {
            if (count <= 0)
                return;
            grain = std::max(grain, 1);
            const int chunks = (count + grain - 1) / grain;
            if (chunks == 1 || threads.empty())
            {
                for (int c = 0; c < chunks; c++)
                    fn(c * grain, std::min(count, (c + 1) * grain), c);
                return;
            }

            // 1) Close the previous job id first: a late worker that still holds an old claim value can no
            //    longer succeed with its compare-exchange, even if it reads the new job's fields below.
            const uint64_t id = ++jobId;
            claim.store((id << 32) | kClosed, std::memory_order_seq_cst);

            // 2) Describe the job.
            using Fn = std::remove_reference_t<F>;
            jobFn = [](void *ctx, int b, int e, int c) { (*static_cast<Fn *>(ctx))(b, e, c); };
            jobCtx = const_cast<void *>(static_cast<const void *>(std::addressof(fn)));
            jobCount = count;
            jobGrain = grain;
            jobChunks.store((uint32_t)chunks, std::memory_order_release);
            completed.store(0, std::memory_order_relaxed);

            // 3) Open it: job id in the high half of the claim counter, next chunk (0) in the low half.
            claim.store(id << 32, std::memory_order_release);
            generation.fetch_add(1, std::memory_order_acq_rel);
            generation.notify_all();

            runChunks(id);

            // wait for the chunks (not for the workers): sleeping workers do not delay us
            while (completed.load(std::memory_order_acquire) < chunks)
                ACCEL_CPU_RELAX();
        }

    private:
        using JobFn = void (*)(void *, int, int, int);

        void runChunks(uint64_t id)
        {
            for (;;)
            {
                uint64_t v = claim.load(std::memory_order_acquire);
                if ((v >> 32) != id)
                    return; // job over, a newer one started
                const uint32_t c = (uint32_t)(v & 0xFFFFFFFFu);
                if (c >= jobChunks.load(std::memory_order_acquire))
                    return; // all chunks taken (or the job is closed: kClosed is larger than any count)
                if (!claim.compare_exchange_weak(v, v + 1, std::memory_order_acq_rel))
                    continue;
                // the job cannot finish while this chunk is unfinished, so its fields are stable here
                const int ci = (int)c;
                jobFn(jobCtx, ci * jobGrain, std::min(jobCount, (ci + 1) * jobGrain), ci);
                completed.fetch_add(1, std::memory_order_acq_rel);
            }
        }

        void workerLoop()
        {
            uint64_t seen = generation.load(std::memory_order_acquire);
            for (;;)
            {
                // spin briefly (physics issues many short jobs per step), then sleep
                uint64_t g = generation.load(std::memory_order_acquire);
                for (int spin = 0; g == seen && spin < 4000; spin++)
                {
                    ACCEL_CPU_RELAX();
                    g = generation.load(std::memory_order_acquire);
                }
                if (g == seen)
                {
                    generation.wait(seen, std::memory_order_acquire);
                    continue;
                }
                seen = g;
                if (quit.load(std::memory_order_acquire))
                    return;
                runChunks(claim.load(std::memory_order_acquire) >> 32);
            }
        }

        std::vector<std::thread> threads;
        std::atomic<uint64_t> generation{0};
        std::atomic<bool> quit{false};
        std::atomic<uint64_t> claim{0}; // (job id << 32) | next chunk
        std::atomic<int> completed{0};
        uint64_t jobId = 0;
        static constexpr uint64_t kClosed = 0xFFFFFFFFu; // chunk index meaning "nothing to claim"

        // current job (written before `claim`/`generation` are published)
        JobFn jobFn = nullptr;
        void *jobCtx = nullptr;
        int jobCount = 0, jobGrain = 1;
        std::atomic<uint32_t> jobChunks{0};
    };
}
