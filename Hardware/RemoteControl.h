#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H

#include <stdint.h>

void RemoteControl_Init(void);
void RemoteControl_Update(uint16_t elapsed_ms);

#endif
