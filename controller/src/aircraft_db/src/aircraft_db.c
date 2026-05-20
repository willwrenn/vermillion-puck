/*
 * SkyWatch controller — aircraft database (Stage 4.1).
 *
 * Direct port of miniproject's `node_db.c`: static pool + sys_slist
 * + k_mutex. Differences from the original:
 *   - Keyed by ICAO string (6 lowercase hex) instead of name.
 *   - 10 s stale window (plan's spec) and a periodic reaper thread.
 *   - No "disabled" flag, no RSSI/path-loss fields.
 *   - Stores a full `struct aircraft_t` per entry; producers fill the
 *     whole struct and call upsert.
 *
 * Fusion policy: freshest observation wins (see resources/standards/fusion.md).
 * Both transports compete for the same row; the upsert overwrites all fields
 * and refreshes `last_seen_ms`. We do not blend.
 */

#include "aircraft_db.h"
#include "kalman.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(aircraft_db, LOG_LEVEL_INF);

/* --- Pool + list + mutex --------------------------------------------- */

static struct aircraft_db_entry s_pool[AIRCRAFT_DB_MAX_ENTRIES];
static bool                     s_pool_used[AIRCRAFT_DB_MAX_ENTRIES];
static sys_slist_t              s_list;
static K_MUTEX_DEFINE(s_mutex);

static struct aircraft_db_stats s_stats;

/* --- Pool helpers (lock must be held) -------------------------------- */



// Purpose: Unit tests for the aircraft_db.c module. These are in the same file
//          as the implementation to keep things simple and self-contained. 
static struct aircraft_db_entry *pool_alloc(void)
{
	for (int i = 0; i < AIRCRAFT_DB_MAX_ENTRIES; i++) {
		if (!s_pool_used[i]) {
			s_pool_used[i] = true;
			memset(&s_pool[i], 0, sizeof(s_pool[i]));
			return &s_pool[i];
		}
	}
	return NULL;
}


//Purpose: Free an entry back to the pool. The caller must have already removed
//		 the entry from the list. The pointer must be from the pool.
static void pool_free(struct aircraft_db_entry *e)
{
	for (int i = 0; i < AIRCRAFT_DB_MAX_ENTRIES; i++) {
		if (&s_pool[i] == e) {
			s_pool_used[i] = false;
			return;
		}
	}
}


// Purpose: Find an entry by ICAO. Caller must hold the mutex. Returns NULL if not found.
static struct aircraft_db_entry *find_locked(const char *icao)
{
	sys_snode_t *cur;
	SYS_SLIST_FOR_EACH_NODE(&s_list, cur) {
		struct aircraft_db_entry *e =
			CONTAINER_OF(cur, struct aircraft_db_entry, node);
		if (strncmp(e->ac.icao, icao, AIRCRAFT_ICAO_BUF_LEN) == 0) {
			return e;
		}
	}
	return NULL;
}

/* --- Public API ------------------------------------------------------- */
//Purpose: Initialize the aircraft database. This zero-initializes the pool and list, 
// and starts the reaper thread. This function is called from main() but could be called multiple times to reset the database.
void aircraft_db_init(void)
{
	sys_slist_init(&s_list);
	memset(s_pool_used, 0, sizeof(s_pool_used));
	memset(s_pool,      0, sizeof(s_pool));
	memset(&s_stats,    0, sizeof(s_stats));
}


// Purpose: Insert or update an aircraft entry. If an entry with the same ICAO exists, 
// it is updated with the new data and last_seen_ms is refreshed. 
// If no entry exists, a new one is allocated from the pool and added to the list.
//  Returns 1 if inserted, 0 if updated, or a negative error code on failure (e.g. -ENOMEM if the pool is full).
int aircraft_db_upsert(const struct aircraft_t *src)
{
	if (!src || src->icao[0] == '\0') {
		return -EINVAL;
	}
	int rc;

	k_mutex_lock(&s_mutex, K_FOREVER);

	int64_t now_ms = k_uptime_get();
	struct aircraft_db_entry *e = find_locked(src->icao);
	bool insert = false;
	if (e) {
		int64_t prev_ms = e->last_seen_ms;
		e->ac           = *src;          /* freshest observation wins */
		e->last_seen_ms = now_ms;
		s_stats.updates++;
		/* Run the KF on the new measurement. */
		double dt = (now_ms - prev_ms) / 1000.0;
		if (dt <= 0.0) dt = 0.001;        /* guard against same-ms updates */
		kalman_predict(&e->kf, dt);
		kalman_update(&e->kf, src->lat, src->lon);
		rc = 0;
	} else {
		e = pool_alloc();
		if (!e) {
			s_stats.pool_full++;
			rc = -ENOMEM;
			goto out;
		}
		e->ac           = *src;
		e->last_seen_ms = now_ms;
		kalman_init(&e->kf, src->lat, src->lon);
		kalman_update(&e->kf, src->lat, src->lon);
		sys_slist_append(&s_list, &e->node);
		s_stats.inserts++;
		insert = true;
		rc = 1;
	}

	/* After the KF update: publish a 10 s-ahead prediction once the filter
	 * has converged (>= AIRCRAFT_PRED_MIN_UPDATES measurements). */
	if (e->kf.update_count >= AIRCRAFT_PRED_MIN_UPDATES) {
		kalman_predict_ahead(&e->kf, AIRCRAFT_PRED_HORIZON_S,
				     &e->ac.pred_lat, &e->ac.pred_lon);
		e->ac.valid_mask |= AIRCRAFT_VALID_PRED;
	}
	(void)insert;

out:
	k_mutex_unlock(&s_mutex);
	return rc;
}

// Purpose: Look up an aircraft by ICAO. If found, copies the data to `out` and returns 0.
// If not found, returns -ENOENT. If invalid arguments, returns -EINVAL.
int aircraft_db_lookup(const char *icao, struct aircraft_t *out)
{
	if (!icao || !out) {
		return -EINVAL;
	}
	int rc = -ENOENT;

	k_mutex_lock(&s_mutex, K_FOREVER);
	struct aircraft_db_entry *e = find_locked(icao);
	if (e) {
		*out = e->ac;
		rc = 0;
	}
	k_mutex_unlock(&s_mutex);
	return rc;
}

// Purpose: Clear all entries from the database. This removes all entries from the list 
// and frees them back to the pool. Also resets stats.
void aircraft_db_clear(void)
{
	k_mutex_lock(&s_mutex, K_FOREVER);
	sys_snode_t *cur, *next;
	SYS_SLIST_FOR_EACH_NODE_SAFE(&s_list, cur, next) {
		struct aircraft_db_entry *e =
			CONTAINER_OF(cur, struct aircraft_db_entry, node);
		sys_slist_find_and_remove(&s_list, &e->node);
		pool_free(e);
	}
	memset(&s_stats, 0, sizeof(s_stats));
	k_mutex_unlock(&s_mutex);
}

// Purpose: Return the current number of entries in the database. 
// This counts the nodes in the list while holding the mutex.
int aircraft_db_count(void)
{
	int n = 0;
	sys_snode_t *cur;
	k_mutex_lock(&s_mutex, K_FOREVER);
	SYS_SLIST_FOR_EACH_NODE(&s_list, cur) {
		n++;
	}
	k_mutex_unlock(&s_mutex);
	return n;
}

// Purpose: Iterate over all entries in the database, calling the provided callback
//  for each one.
void aircraft_db_for_each(aircraft_db_cb_t cb, void *user)
{
	if (!cb) return;
	sys_snode_t *cur, *next;
	k_mutex_lock(&s_mutex, K_FOREVER);
	SYS_SLIST_FOR_EACH_NODE_SAFE(&s_list, cur, next) {
		struct aircraft_db_entry *e =
			CONTAINER_OF(cur, struct aircraft_db_entry, node);
		if (!cb(e, user)) {
			break;
		}
	}
	k_mutex_unlock(&s_mutex);
}

// Purpose: Get current stats about the database, such as number of inserts,
//  updates, evictions, etc.
void aircraft_db_get_stats(struct aircraft_db_stats *out)
{
	k_mutex_lock(&s_mutex, K_FOREVER);
	*out = s_stats;
	k_mutex_unlock(&s_mutex);
}

/* --- Stale reaper thread --------------------------------------------- */

#define REAPER_STACK 1024
#define REAPER_PRIO  8
#define REAPER_TICK_MS 2000

static K_THREAD_STACK_DEFINE(reaper_stack, REAPER_STACK);
static struct k_thread       reaper_thread;
// Purpose: This function is run by the reaper thread. It iterates over all entries 
// in the database and evicts any that have not been seen for more than 
// AIRCRAFT_DB_STALE_MS milliseconds. The caller must hold the mutex.
static void reap_locked(int64_t now_ms)
{
	sys_snode_t *cur, *next, *prev = NULL;
	SYS_SLIST_FOR_EACH_NODE_SAFE(&s_list, cur, next) {
		struct aircraft_db_entry *e =
			CONTAINER_OF(cur, struct aircraft_db_entry, node);
		if ((now_ms - e->last_seen_ms) > AIRCRAFT_DB_STALE_MS) {
			sys_slist_remove(&s_list, prev, cur);
			LOG_INF("stale evict %s (age %lld ms, src=%s)",
				e->ac.icao,
				(long long)(now_ms - e->last_seen_ms),
				aircraft_source_str(e->ac.source));
			pool_free(e);
			s_stats.stale_evicts++;
			/* prev stays — `cur` was unlinked. */
		} else {
			prev = cur;
		}
	}
}

// Purpose: Spawn the reaper thread. This is called at SYS_INIT after the kernel is up. 
// The thread runs indefinitely, sleeping for REAPER_TICK_MS milliseconds between iterations.
static void reaper_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_MSEC(REAPER_TICK_MS));
		k_mutex_lock(&s_mutex, K_FOREVER);
		reap_locked(k_uptime_get());
		k_mutex_unlock(&s_mutex);
	}
}


// Purpose: Spawn the reaper thread. This is called at SYS_INIT after the kernel is up.
static int reaper_spawn(void)
{
	k_thread_create(&reaper_thread, reaper_stack,
			K_THREAD_STACK_SIZEOF(reaper_stack),
			reaper_thread_fn, NULL, NULL, NULL,
			REAPER_PRIO, 0, K_NO_WAIT);
	k_thread_name_set(&reaper_thread, "db_reaper");
	return 0;
}

/* Auto-start reaper at SYS_INIT after kernel is up. The DB itself doesn't
 * need explicit init from main() — sys_slist_init zero-initialises fine
 * via the .data section, but call aircraft_db_init() explicitly anyway to
 * keep init order obvious. */
SYS_INIT(reaper_spawn, APPLICATION, 90);
