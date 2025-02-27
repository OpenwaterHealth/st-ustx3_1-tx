/*
 * module_manager.h
 *
 *  Created on: Feb 17, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_MODULE_MANAGER_H_
#define INC_MODULE_MANAGER_H_


#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"  // Adjust include as needed for your MCU

// Configuration: each module has two transmitters; adjust as necessary.
#define TX_PER_MODULE 2
#define MAX_MODULES   6  // Total number of modules (master + slaves)

// Represents a transmitter (TX7332)
typedef struct {
    GPIO_TypeDef* cs_port;  // Chip-select port
    uint16_t cs_pin;        // Chip-select pin
} TX7332;

// Represents a module (master or slave) that holds TX7332 devices.
typedef struct {
    uint8_t i2c_address;          // I2C address (for master, this may be unused or fixed)
    uint8_t num_transmitters;     // Number of transmitters (typically TX_PER_MODULE)
    TX7332 transmitters[TX_PER_MODULE];
} ModuleInfo;

/**
 * @brief  Returns the module index for a given global transmitter index.
 *         Global indexes are assigned sequentially: e.g., 0,1 for master,
 *         2,3 for first slave, etc.
 *
 * @param  globalTxIndex Global transmitter index.
 * @return uint8_t Module index.
 */
uint8_t ModuleManager_GetModuleIndex(uint8_t globalTxIndex);

/**
 * @brief  Returns the local transmitter index within its module.
 *
 * @param  globalTxIndex Global transmitter index.
 * @return uint8_t Local index within the module.
 */
uint8_t ModuleManager_GetLocalTxIndex(uint8_t globalTxIndex);

/**
 * @brief  Initializes the module manager and clears all module entries.
 */
void ModuleManager_Init(void);

/**
 * @brief  Deinitializes the module manager by clearing the internal module storage.
 *         Call this when the USB is unplugged or a role change is required.
 */
void ModuleManager_DeInit(void);

/**
 * @brief  Registers the master module.
 *
 * @param  i2c_address I2C address for the master module (if applicable)
 * @return int 0 on success, negative value on error.
 */
int ModuleManager_RegisterMaster(uint8_t i2c_address);

/**
 * @brief  Adds a slave module (discovered over I2C).
 *
 * @param  i2c_address I2C address of the slave module.
 * @return int The index of the newly added module on success, or negative value on error.
 */
int ModuleManager_AddSlave(uint8_t i2c_address);

/**
 * @brief  Gets a pointer to a registered module by its index.
 *
 * @param  moduleIndex Index of the module.
 * @return ModuleInfo* Pointer to the module, or NULL if index is out-of-range.
 */
ModuleInfo* ModuleManager_GetModule(uint8_t moduleIndex);

/**
 * @brief  Retrieves a pointer to a TX7332 transmitter based on its global transmitter index.
 *
 * @param  globalTxIndex Global transmitter index.
 * @return TX7332* Pointer to the transmitter structure, or NULL if not found.
 */
TX7332* ModuleManager_GetTransmitter(uint8_t globalTxIndex);

#ifdef __cplusplus
}
#endif

#endif /* INC_MODULE_MANAGER_H_ */
