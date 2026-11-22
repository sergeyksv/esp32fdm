#pragma once
/* Log lines go to sim_log() so tests can look for them ("giving up", ...) */
void sim_log(char level, const char *tag, const char *fmt, ...) __attribute__((format(printf, 3, 4)));
#define ESP_LOGE(tag, ...) sim_log('E', tag, __VA_ARGS__)
#define ESP_LOGW(tag, ...) sim_log('W', tag, __VA_ARGS__)
#define ESP_LOGI(tag, ...) sim_log('I', tag, __VA_ARGS__)
#define ESP_LOGD(tag, ...) ((void)0)
#define ESP_LOGV(tag, ...) ((void)0)
