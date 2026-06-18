#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

void AppDebug_Init(void);
void AppDebug_Printf(const char* fmt, ...);
void AppDebug_AssertFailed(const char* file, int line);

#ifdef __cplusplus
}
#endif

#endif /* APP_DEBUG_H */
