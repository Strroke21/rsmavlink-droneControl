extern crate mavlink;

use mavlink::{MavHeader, MavConnection, common::MavMessage, common::MavModeFlag};
use mavlink::common::{MavFrame, SET_POSITION_TARGET_LOCAL_NED_DATA, PositionTargetTypemask};
use std::time::Duration;
use std::thread;
use std::error::Error;


fn connect(connection_string: &str) -> Result<MavlinkConnection<mavlink::common::MavMessage>, Box<dyn Error>> {
    // Establish MAVLink connection using the provided connection string
    let vehicle = mavlink::connect(connection_string)?;

    Ok(vehicle)
}

fn enable_data_stream(
    vehicle: &mut MavlinkConnection<MavMessage>,
    stream_rate: u32,
) -> Result<(), Box<dyn Error>> {
    // Wait for a heartbeat to ensure the connection is alive
    vehicle.wait_heartbeat()?;

    // Send the request to enable data stream
    vehicle.mav.request_data_stream_send(
        vehicle.target_system,
        vehicle.target_component,
        MAV_DATA_STREAM_ALL as u32,
        stream_rate,
        1, // Requesting data stream (1: start sending)
    )?;

    println!("Data stream enabled with rate: {}", stream_rate);
    Ok(())
}


fn arm(vehicle: &mut MavlinkConnection<MavMessage>) -> Result<(), Box<dyn Error>> {
    // Send the command to arm the drone (MAV_CMD_COMPONENT_ARM_DISARM)
    vehicle.mav.command_long_send(
        vehicle.target_system,
        vehicle.target_component,
        MAV_CMD_COMPONENT_ARM_DISARM,
        0,  // Confirmation (0: no confirmation)
        1,  // Arm the drone (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0, 0, 0,
    )?;

    println!("Vehicle armed.");
    Ok(())
}


fn velocity_command(
    vehicle: &mut dyn MavConnection<MavMessage>,  // Specify MavMessage
    vx: f32, vy: f32, vz: f32,
) -> Result<(), mavlink::error::MessageWriteError> {
    let msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(
        SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0,                     // time_boot_ms (timestamp)
            target_system: 1,                     // Hard-code target system ID
            target_component: 1,                  // Hard-code target component ID
            coordinate_frame: MavFrame::MAV_FRAME_LOCAL_NED, // Frame
            type_mask: PositionTargetTypemask::from_bits_truncate(0b0000111111000111), // Type mask (only velocity is used)
            x: 0.0, y: 0.0, z: 0.0,              // Position (unused in this case)
            vx: vx as f32, vy: vy as f32, vz: vz as f32,  // Velocity (passed as parameters)
            afx: 0.0, afy: 0.0, afz: 0.0,         // Acceleration (unused)
            yaw: 0.0, yaw_rate: 0.0,              // Yaw and yaw rate (unused)
        }
    );

    let header = MavHeader {
        system_id: 1, 
        component_id: 1,  
        sequence: 0,   
    };

    vehicle.send(&header, &msg)?;

    Ok(())
}

fn send_position_setpoint(
    vehicle: &mut MavlinkConnection<MavMessage>,
    target_system: u8,
    target_component: u8,
    x: f32, // X position in meters (ENU or NED)
    y: f32, // Y position in meters (ENU or NED)
    z: f32, // Z position in meters (positive altitude in ENU or negative in NED)
    yaw: f32, // Yaw angle in radians
) -> Result<(), Box<dyn Error>> {
    // Create a MAVLink SET_POSITION_TARGET_LOCAL_NED message
    let position_msg = MavMessage::SET_POSITION_TARGET_LOCAL_NED(
        mavlink::common::SET_POSITION_TARGET_LOCAL_NED_DATA {
            time_boot_ms: 0, // Can be 0 unless you want to timestamp it
            target_system,
            target_component,
            coordinate_frame: mavlink::common::MavFrame::MAV_FRAME_LOCAL_NED as u8,
            type_mask: 0b0000111111000111, // Set position and yaw, ignore velocity, acceleration, etc.
            x,
            y,
            z,
            vx: 0.0,
            vy: 0.0,
            vz: 0.0,
            afx: 0.0,
            afy: 0.0,
            afz: 0.0,
            yaw,
            yaw_rate: 0.0,
        },
    );

    // Send the message to the vehicle
    vehicle.send(&position_msg)?;
    println!("Position setpoint sent: x={}, y={}, z={}, yaw={}", x, y, z, yaw);
    Ok(())
}


fn get_global_position(
    vehicle: &mut MavlinkConnection<MavMessage>,
) -> Result<Option<[f64; 7]>, Box<dyn Error>> {
    // Wait for a GLOBAL_POSITION_INT message
    if let MavMessage::GLOBAL_POSITION_INT(data) = vehicle.recv()? {
        // Convert the data into an array: [lat, lon, alt, relative_alt, vx, vy, vz]
        let position = [
            data.lat as f64 / 1e7,          // Latitude in degrees
            data.lon as f64 / 1e7,          // Longitude in degrees
            data.alt as f64 / 1000.0,       // Altitude in meters
            data.relative_alt as f64 / 1000.0, // Relative altitude in meters
            data.vx as f64 / 100.0,         // Velocity X in m/s
            data.vy as f64 / 100.0,         // Velocity Y in m/s
            data.vz as f64 / 100.0,         // Velocity Z in m/s
        ];
        Ok(Some(position))
    } else {
        // No GLOBAL_POSITION_INT message received
        Ok(None)
    }
}


fn get_local_position(
    vehicle: &mut MavlinkConnection<MavMessage>,
) -> Result<Option<[f64; 3]>, Box<dyn Error>> {
    // Wait for a LOCAL_POSITION_NED message
    if let MavMessage::LOCAL_POSITION_NED(data) = vehicle.recv()? {
        // Convert the data into an array: [x, y, z]
        let position = [
            data.x as f64, // X position in meters
            data.y as f64, // Y position in meters
            data.z as f64, // Z position in meters (negative for altitude in NED frame)
        ];
        Ok(Some(position))
    } else {
        // No LOCAL_POSITION_NED message received
        Ok(None)
    }
}

fn VehicleMode(
    vehicle: &mut MavlinkConnection<MavMessage>,
    mode: &str,
) -> Result<(), Box<dyn Error>> {
    // List of possible modes
    let modes = [
        "STABILIZE", "ACRO", "ALT_HOLD", "AUTO", "GUIDED", "LOITER", "RTL", "CIRCLE", "", "LAND"
    ];

    // Find the mode id
    let mode_id = modes.iter().position(|&m| m == mode).unwrap_or(12); // Default to 12 if not found

    // Send the SET_MODE command with the desired mode id
    vehicle.mav.set_mode_send(
        vehicle.target_system,
        mavlink::common::MAV_MODE_FLAG_CUSTOM_MODE_ENABLED as u32,
        mode_id as u32,
    )?;

    println!("Vehicle mode set to: {}", mode);
    Ok(())
}


