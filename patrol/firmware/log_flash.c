#include "journal.h"
#include "driverlib/flash.h"

extern uint8_t _event_log_start[];
bool log_erase_page(unsigned slot) {
    return FlashErase((uint32_t)_event_log_start + slot * sizeof(Event)) == 0;
}
bool log_program(unsigned slot, const Event *event) {
    uint32_t words[] = {event->id, event->ms, event->code, event->check};
    return FlashProgram(words, (uint32_t)_event_log_start + slot * sizeof(Event), sizeof words) == 0;
}
