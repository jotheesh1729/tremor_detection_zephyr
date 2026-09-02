#ifndef TREMOR_BLE_SERVICE_H_
#define TREMOR_BLE_SERVICE_H_

#include "episode_fsm.h"

/* Aggregated stats since the last report, not a live per-window snapshot --
 * radio traffic for a battery wearable should be periodic, not continuous.
 */
struct tremor_report {
	uint8_t tremor_episodes;
	uint8_t dyskinesia_episodes;
	uint16_t tremor_duration_s;
	uint16_t dyskinesia_duration_s;
	uint8_t mean_severity_pct; /* mean over episodes in this period, 0-100 */
};

/* Starts advertising and registers the GATT service. Call once from main(). */
void ble_service_start(void);

/* Notifies the report characteristic. No-op if nothing is subscribed. */
void ble_service_notify_report(const struct tremor_report *report);

/* Notifies the episode characteristic with a closed episode record.
 * No-op if nothing is subscribed.
 */
void ble_service_notify_episode(const struct episode_record *record);

#endif /* TREMOR_BLE_SERVICE_H_ */
