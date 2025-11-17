# MQTT Configuration

This directory contains the custom Mosquitto configuration file:
my_mosquitto.conf.

## Configuration Overview

### my_mosquitto.conf
```conf
listener 1883
protocol mqtt

listener 9001
protocol websockets

allow_anonymous true
```

## Explanation:

Port 1883 – standard MQTT broker endpoint.

Port 9001 – WebSocket-compatible MQTT endpoint (e.g. for browser clients).

allow_anonymous true – no authentication required.
(Use only for local development.)

## How to Run the Broker

Run Mosquitto using this configuration:

```bash
mosquitto -v -c my_mosquitto.conf
```

- -v enables verbose logging
- -c selects the config file

The broker will start and open both listeners.

## How It Works

Mosquitto reads my_mosquitto.conf and starts two network listeners.

Any MQTT client (e.g. mosquitto_pub, mosquitto_sub, your robot software, or the Jetson) can connect to port 1883.

Web applications or tools using MQTT-over-WebSockets can connect to port 9001.

Because anonymous access is enabled, clients can publish/subscribe without usernames or passwords.
