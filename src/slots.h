// Persistent 4-slot multi-controller pairing storage. Stores the 4 bonded
// DS5 bd_addrs in a custom flash sector (BTstack's NVM keeps the link_keys).
//
// Multi-slot pairing modeled on zurce/DS5Dongle-OLED. Credit to zurce.

#ifndef DS5_BRIDGE_SLOTS_H
#define DS5_BRIDGE_SLOTS_H

#include <cstdint>

constexpr int kNumSlots = 4;
// User-given controller label per slot ("Volcanic Red"), 12 chars + NUL.
// Set over USB (set_ds5.py --slot-set / 0xF6 func 0x11); shown on the Slots
// screens and dumped via the 0xFA feature report for fleet bookkeeping.
constexpr int kSlotNameLen = 12;

void slots_load();
bool slot_occupied(int slot);
void slot_get_addr(int slot, uint8_t out[6]);
int  slot_owner_of(const uint8_t addr[6]);
void slot_assign(int slot, const uint8_t addr[6]);
void slot_forget(int slot);
void slots_wipe_all();
bool slots_any_empty();

// Slot names (8-Player fleet bookkeeping). Getter always NUL-terminates into
// out[kSlotNameLen+1]; empty string = unnamed. Setter clamps/pads and persists.
// slot_set_entry provisions addr + name + occupied in one flash write — used
// by the USB slot-write op to rebuild a slot table from a saved database
// (note: the BT link key still lives in BTstack NVM; a provisioned addr whose
// key was never bonded on this dongle needs one Create+PS pairing pass).
void slot_get_name(int slot, char out[kSlotNameLen + 1]);
void slot_set_name(int slot, const char *name);
void slot_set_entry(int slot, const uint8_t addr[6], const char *name);

#endif // DS5_BRIDGE_SLOTS_H
