// license:BSD-3-Clause
// Timestamped card input transport. The producer never changes a published
// entry; release/acquire transfers its ownership to the single consumer.
#ifndef MAME_BUS_A2BUS_SECONDSIGHT_QUEUE_H
#define MAME_BUS_A2BUS_SECONDSIGHT_QUEUE_H
#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#if defined(__APPLE__)
#include <dispatch/dispatch.h>
#endif

template <typename T, unsigned Capacity> class secondsight_queue
{
	static_assert(Capacity && !(Capacity & (Capacity - 1)));

  public:
	secondsight_queue()
		: m_entries(std::make_unique<std::array<T, Capacity>>())
	{
	}

	~secondsight_queue()
	{
#if defined(__APPLE__)
		dispatch_release(m_park);
#endif
	}

	bool push(T const &entry, bool wake = true)
	{
		const auto write = m_write.load(std::memory_order_relaxed);
		const auto read = m_read.load(std::memory_order_acquire);
		if (write - read == Capacity)
			return false;
		(*m_entries)[write & (Capacity - 1)] = entry;
		m_write.store(write + 1, std::memory_order_release);
		// Publish every entry immediately. Only wake a sleeping consumer at a
		// queued batch or an explicit fence; no emulated input is combined.
		// Inline service can drain earlier entries, so a lifetime write count
		// would wake the worker for a single edge after an idle rendezvous.
		if (wake || write + 1 - read >= 64)
			notify();
		return true;
	}

	void notify()
	{
#if defined(__APPLE__)
		if (m_sleeping.exchange(false, std::memory_order_seq_cst))
			dispatch_semaphore_signal(m_park);
#else
		m_wakeup.fetch_add(1, std::memory_order_release);
		m_wakeup.notify_one();
#endif
	}

	// The consumer may migrate, but only under this ownership token.
	bool claim()
	{
		bool idle = false;
		return m_owned.compare_exchange_strong(idle, true, std::memory_order_acquire);
	}

	void release()
	{
		m_owned.store(false, std::memory_order_release);
		m_owned.notify_one();
	}

	void wait_owner() { m_owned.wait(true, std::memory_order_relaxed); }

	bool take(T &entry)
	{
		const auto read = m_read.load(std::memory_order_relaxed);
		if (read == m_write.load(std::memory_order_acquire))
			return false;
		entry = (*m_entries)[read & (Capacity - 1)];
		// Only the token-owning consumer changes this index. Publishing the
		// next value needs release ordering, not a read/modify/write operation.
		m_read.store(read + 1, std::memory_order_release);
		return true;
	}

	void wait_available()
	{
		while (m_read.load(std::memory_order_acquire) == m_write.load(std::memory_order_acquire))
		{
#if defined(__APPLE__)
			m_sleeping.exchange(true, std::memory_order_seq_cst);
			if (m_read.load(std::memory_order_acquire) != m_write.load(std::memory_order_acquire))
			{
				m_sleeping.exchange(false, std::memory_order_seq_cst);
				break;
			}
			dispatch_semaphore_wait(m_park, DISPATCH_TIME_FOREVER);
#else
			const auto wakeup = m_wakeup.load(std::memory_order_acquire);
			if (m_read.load(std::memory_order_acquire) != m_write.load(std::memory_order_acquire))
				break;
			m_wakeup.wait(wakeup, std::memory_order_relaxed);
#endif
		}
	}

	T const &front(unsigned spins = 0)
	{
		const auto read = m_read.load(std::memory_order_relaxed);
		// A response fence is commonly followed by another short CPU request.
		// Poll only that handoff, without libc++'s repeated host-clock reads.
		while (spins-- && read == m_write.load(std::memory_order_acquire))
			std::atomic_signal_fence(std::memory_order_seq_cst);
		while (read == m_write.load(std::memory_order_acquire))
		{
#if defined(__APPLE__)
			// libc++ atomic::wait spins on a clock before parking. This queue
			// is deliberately lazy: an empty worker should consume no CPU.
			// The sequentially consistent handshake prevents a publication
			// being lost between the empty check and the semaphore wait.
			m_sleeping.exchange(true, std::memory_order_seq_cst);
			if (read != m_write.load(std::memory_order_acquire))
			{
				m_sleeping.exchange(false, std::memory_order_seq_cst);
				break;
			}
			dispatch_semaphore_wait(m_park, DISPATCH_TIME_FOREVER);
#else
			const auto wakeup = m_wakeup.load(std::memory_order_acquire);
			if (read != m_write.load(std::memory_order_acquire))
				break;
			m_wakeup.wait(wakeup, std::memory_order_relaxed);
#endif
		}
		return (*m_entries)[read & (Capacity - 1)];
	}

	void pop() { m_read.store(m_read.load(std::memory_order_relaxed) + 1, std::memory_order_release); }

	std::uint64_t published() const { return m_write.load(std::memory_order_acquire); }

	std::uint64_t consumed() const { return m_read.load(std::memory_order_acquire); }

  private:
	std::unique_ptr<std::array<T, Capacity>> m_entries;
	// Keep producer, consumer and ownership traffic separate on hosts with
	// 128-byte cache lines as well as those with 64-byte lines.
	alignas(128) std::atomic<bool> m_owned{false};
	alignas(128) std::atomic<std::uint64_t> m_write{0};
	alignas(128) std::atomic<std::uint64_t> m_read{0};
#if defined(__APPLE__)
	alignas(128) std::atomic<bool> m_sleeping{false};
	dispatch_semaphore_t m_park = dispatch_semaphore_create(0);
#else
	alignas(128) std::atomic<std::uint64_t> m_wakeup{0};
#endif
};

#endif
