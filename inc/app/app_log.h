#ifndef APP_LOG_H
#define APP_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    APP_LOG_LEVEL_DEBUG = 0,
    APP_LOG_LEVEL_INFO,
    APP_LOG_LEVEL_WARN,
    APP_LOG_LEVEL_ERROR,
    APP_LOG_LEVEL_OFF,
} AppLogLevel;

void AppLog_SetLevel(AppLogLevel level);
AppLogLevel AppLog_GetLevel(void);
void AppLog_Print(AppLogLevel level, const char* module, const char* fmt, ...);

#define APP_LOGD(module, fmt, ...) AppLog_Print(APP_LOG_LEVEL_DEBUG, (module), (fmt), ##__VA_ARGS__)
#define APP_LOGI(module, fmt, ...) AppLog_Print(APP_LOG_LEVEL_INFO, (module), (fmt), ##__VA_ARGS__)
#define APP_LOGW(module, fmt, ...) AppLog_Print(APP_LOG_LEVEL_WARN, (module), (fmt), ##__VA_ARGS__)
#define APP_LOGE(module, fmt, ...) AppLog_Print(APP_LOG_LEVEL_ERROR, (module), (fmt), ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
