#pragma once

#include <stdint.h>

// Frames the LilyGO can use during normal operation.  Keeping this list on the
// satellite prevents a saturated Vehicle CAN from flooding the UART and the
// LilyGO control loop.  A CAN2 HTTP capture explicitly switches the satellite
// to forward-all, so diagnostics still see the complete bus.
static inline bool remote_can_operational_id(uint32_t id) {
    switch (id) {
        case 0x045u: // STW_ACTN_RQ
        case 0x082u: // UI_tripPlanning
        case 0x102u: // VCLEFT_doorStatus
        case 0x119u: // VCSEC_windowRequests
        case 0x129u: // steering angle
        case 0x132u: // BMS_hvBusStatus
        case 0x145u: // ESP_status
        case 0x229u: // right stalk / gear lever
        case 0x238u: // UI_driverAssistMapData
        case 0x273u: // UI_vehicleControl
        case 0x292u: // BMS_socStatus
        case 0x2B9u: // DAS_control
        case 0x312u: // BMS_thermalStatus
        case 0x318u: // GTW_carState
        case 0x331u: // DAS autopilot config
        case 0x353u: // UI_autopilotTrial
        case 0x370u: // EPAS status / AntiNag
        case 0x389u: // DAS_status2
        case 0x398u: // GTW_carConfig
        case 0x399u: // DAS_status / ISA
        case 0x39Bu: // HW4 DAS_status
        case 0x3C2u: // observed Juniper AP/vehicle-state candidate
        case 0x3E9u: // DAS_bodyControls
        case 0x3EEu: // legacy autopilot control
        case 0x3F5u: // VCFRONT_lighting
        case 0x3F8u: // UI_driverAssistControl
        case 0x3FDu: // DAS_autopilotControl
            return true;
        default:
            return false;
    }
}
