#ifndef TREMOR_BLE_SERVICE_H_
#define TREMOR_BLE_SERVICE_H_

#include "dsp/pipeline.h"
#include "episode_fsm.h"

/* Starts advertising and registers the GATT service. Call once from main(). */
void ble_service_start(void);

/* Notifies the severity characteristic with the latest window result.
 * No-op if nothing is subscribed.
 */
void ble_service_notify_severity(const struct tremor_window_result *result);

/* Notifies the episode characteristic with a closed episode record.
 * No-op if nothing is subscribed.
 */
void ble_service_notify_episode(const struct episode_record *record);

#endif /* TREMOR_BLE_SERVICE_H_ */
