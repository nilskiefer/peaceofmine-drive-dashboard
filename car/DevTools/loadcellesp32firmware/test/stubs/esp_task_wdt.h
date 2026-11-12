#pragma once
#include <cassert>
#include <cstdint>
using esp_err_t = int;
constexpr int ESP_OK = 0, ESP_ERR_INVALID_STATE = 1;
struct esp_task_wdt_config_t {
  uint32_t timeout_ms;
  uint32_t idle_core_mask;
  bool trigger_panic;
};
inline bool watchdogInitialized = false, watchdogSubscribed = false;
inline unsigned int watchdogFeeds = 0;
inline esp_task_wdt_config_t watchdogConfig = {};
inline esp_err_t esp_task_wdt_init(const esp_task_wdt_config_t *config) {
  if (watchdogInitialized) return ESP_ERR_INVALID_STATE;
  watchdogInitialized = true; watchdogConfig = *config; return ESP_OK;
}
inline esp_err_t esp_task_wdt_reconfigure(const esp_task_wdt_config_t *config) {
  watchdogConfig = *config; return ESP_OK;
}
inline esp_err_t esp_task_wdt_status(void *) { return watchdogSubscribed ? ESP_OK : 2; }
inline esp_err_t esp_task_wdt_add(void *) { watchdogSubscribed = true; return ESP_OK; }
inline esp_err_t esp_task_wdt_reset() { assert(watchdogSubscribed); ++watchdogFeeds; return ESP_OK; }
#define ESP_ERROR_CHECK(result) assert((result) == ESP_OK)
