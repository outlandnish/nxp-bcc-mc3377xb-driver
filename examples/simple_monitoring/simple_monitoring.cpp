/**
 * Simple Battery Cell Monitoring Example
 *
 * This example demonstrates basic usage of the BatteryCellController library
 * to monitor cell voltages, stack voltage, and IC temperature from a single
 * or multiple MC33772B devices in a TPL chain.
 *
 * Hardware Setup:
 * - MC33772B Battery Cell Controller(s) (6 cells per device)
 * - MC33664 TPL Transceiver
 * - STM32F4 microcontroller
 *
 * Wiring:
 * - BCC_TX_CS to TPL transmit chip select
 * - BCC_RX_CS to TPL receive chip select
 * - BCC_TX_DATA, BCC_TX_SCK to TPL transmit SPI
 * - BCC_RX_DATA, BCC_RX_SCK to TPL receive SPI
 * - BCC_ENABLE to MC33664 enable pin
 * - BCC_INTB to MC33772B interrupt pin
 */

#include <Arduino.h>
#include <SPI.h>
#include "BatteryCellController.h"
#include "TPLSPI.h"

// Pin definitions - adjust these to match your hardware
#define BCC_TX_SCK      PA5
#define BCC_TX_CS       PA6
#define BCC_TX_DATA     PA7
#define BCC_RX_SCK      PA9
#define BCC_RX_CS       PB9
#define BCC_RX_DATA     PA10
#define BCC_ENABLE_PIN  PE8
#define BCC_INTB_PIN    PE9

// Configuration
#define MAX_DEVICES 4      // Number of MC33772B devices in the chain
#define CELL_COUNT 6       // Number of cells per device (MC33772B has 6)
#define MEASUREMENT_INTERVAL_MS 1000  // Read measurements every 1 second

// Global objects
SPIClass *tx_spi, *rx_spi;
TPLSPI *tpl;
BatteryCellController *bcc;
bcc_device_t devices[MAX_DEVICES];

void setup() {
    // Initialize serial for output
    Serial.begin(115200);
    delay(2000);  // Wait for serial connection

    Serial.println("========================================");
    Serial.println("  BCC Simple Monitoring Example");
    Serial.println("========================================\n");

    // Configure device types (all MC33772B in this example)
    for (uint8_t i = 0; i < MAX_DEVICES; i++) {
        devices[i] = BCC_DEVICE_MC33772;
    }

    // Initialize TX and RX SPI interfaces
    Serial.println("Initializing SPI interfaces...");
    tx_spi = new SPIClass(BCC_TX_DATA, NC, BCC_TX_SCK, NC);
    rx_spi = new SPIClass(BCC_RX_DATA, NC, BCC_RX_SCK, BCC_RX_CS);

    // Initialize TPL interface
    Serial.println("Initializing TPL interface...");
    tpl = new TPLSPI(tx_spi, rx_spi, BCC_TX_CS);

    // Create BCC controller instance
    Serial.println("Creating BCC controller...");
    bcc = new BatteryCellController(
        tpl,                    // TPL interface
        devices,                // Device type array
        MAX_DEVICES,            // Number of devices in chain
        CELL_COUNT,             // Cells per device
        BCC_ENABLE_PIN,         // Enable pin
        BCC_INTB_PIN,           // Interrupt pin
        false                   // Loopback mode disabled
    );

    // Setup CS pin
    pinMode(BCC_TX_CS, OUTPUT);
    digitalWrite(BCC_TX_CS, HIGH);

    // Initialize the BCC chain
    Serial.println("Initializing BCC chain...");
    bcc_status_t status = bcc->begin();

    if (status == BCC_STATUS_SUCCESS) {
        Serial.println("BCC initialization successful!\n");
    } else {
        Serial.printf("BCC initialization failed with error: %d\n", status);
        Serial.println("Please check your hardware connections and reset.");
        while(1) {
            delay(1000);  // Halt on initialization failure
        }
    }

    Serial.println("Starting measurements...\n");
}

void loop() {
    // Read measurements from each device in the chain
    for (uint8_t dev = 0; dev < MAX_DEVICES; dev++) {
        bcc_cid_t cid = static_cast<bcc_cid_t>(dev + 1);  // CID starts at 1

        Serial.printf("========== Device %d ==========\n", cid);

        // Read cell voltages
        uint32_t cell_voltages[CELL_COUNT];
        bcc_status_t status = bcc->get_cell_voltages(cid, cell_voltages);

        if (status == BCC_STATUS_SUCCESS) {
            Serial.println("Cell Voltages:");
            float total_voltage = 0.0f;

            for (uint8_t i = 0; i < CELL_COUNT; i++) {
                float voltage_v = cell_voltages[i] / 1000000.0f;  // Convert uV to V
                total_voltage += voltage_v;
                Serial.printf("  Cell %d: %.4f V\n", i + 1, voltage_v);
            }

            Serial.printf("  Total: %.4f V\n\n", total_voltage);
        } else {
            Serial.printf("  Error reading cell voltages: %d\n\n", status);
        }

        // Read stack voltage (alternative method using BCC's internal calculation)
        uint32_t stack_voltage_uv;
        status = bcc->get_stack_voltage(cid, &stack_voltage_uv);

        if (status == BCC_STATUS_SUCCESS) {
            float stack_v = stack_voltage_uv / 1000000.0f;
            Serial.printf("Stack Voltage: %.4f V\n", stack_v);
        } else {
            Serial.printf("Error reading stack voltage: %d\n", status);
        }

        // Read IC temperature
        int16_t temperature_c;
        status = bcc->get_ic_temperature(cid, BCC_TEMP_CELSIUS, &temperature_c);

        if (status == BCC_STATUS_SUCCESS) {
            Serial.printf("IC Temperature: %d C\n", temperature_c);
        } else {
            Serial.printf("Error reading temperature: %d\n", status);
        }

        // Read current sense voltage (if using shunt)
        int32_t current_sense_uv;
        status = bcc->get_current_sense_voltage(cid, &current_sense_uv);

        if (status == BCC_STATUS_SUCCESS) {
            float current_sense_mv = current_sense_uv / 1000.0f;
            Serial.printf("Current Sense: %.2f mV\n", current_sense_mv);
        } else {
            Serial.printf("Error reading current sense: %d\n", status);
        }

        Serial.println();
    }

    Serial.println("========================================\n");

    // Wait before next measurement cycle
    delay(MEASUREMENT_INTERVAL_MS);
}
