#include "module_manager.h"

// Internal storage for module information.
static ModuleInfo modules[MAX_MODULES];
static uint8_t totalModules = 0;

static void ModuleManager_clear_storage() {
    totalModules = 0;
    // Clear all module entries.
    for (int i = 0; i < MAX_MODULES; i++) {
        modules[i].i2c_address = 0;
        modules[i].num_transmitters = 0;
        for (int j = 0; j < TX_PER_MODULE; j++) {
            modules[i].transmitters[j].cs_port = NULL;
            modules[i].transmitters[j].cs_pin = 0;
        }
    }
}

uint8_t ModuleManager_GetModuleIndex(uint8_t globalTxIndex) {
    return globalTxIndex / TX_PER_MODULE;
}

uint8_t ModuleManager_GetLocalTxIndex(uint8_t globalTxIndex) {
    return globalTxIndex % TX_PER_MODULE;
}

void ModuleManager_Init(void) {
	ModuleManager_clear_storage();
}

void ModuleManager_DeInit(void) {
	ModuleManager_clear_storage();
}

int ModuleManager_RegisterMaster(uint8_t i2c_address) {
    if (totalModules >= MAX_MODULES) {
        return -1;  // No space available.
    }
    // The master module is stored at index 0.
    modules[0].i2c_address = i2c_address;
    modules[0].num_transmitters = TX_PER_MODULE;
    // You might want to initialize the transmitters here using your TX7332_Init() function if needed.
    totalModules = 1;
    return 0;
}

int ModuleManager_AddSlave(uint8_t i2c_address) {
    if (totalModules >= MAX_MODULES) {
        return -1;  // Maximum number of modules reached.
    }
    modules[totalModules].i2c_address = i2c_address;
    modules[totalModules].num_transmitters = TX_PER_MODULE;
    // Initialization of each TX7332 on the slave may be done here or later.
    totalModules++;
    return totalModules - 1;  // Return the index of the newly added module.
}

ModuleInfo* ModuleManager_GetModule(uint8_t moduleIndex) {
    if (moduleIndex >= totalModules) {
        return NULL;
    }
    return &modules[moduleIndex];
}

TX7332* ModuleManager_GetTransmitter(uint8_t globalTxIndex) {
    uint8_t moduleIndex = ModuleManager_GetModuleIndex(globalTxIndex);
    uint8_t localTxIndex = ModuleManager_GetLocalTxIndex(globalTxIndex);
    if (moduleIndex >= totalModules) {
        return NULL;
    }
    ModuleInfo* mod = &modules[moduleIndex];
    if (localTxIndex >= mod->num_transmitters) {
        return NULL;
    }
    return &mod->transmitters[localTxIndex];
}
