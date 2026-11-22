/*
 * ESP32-C6 hot tub controller (parent driver)
 *
 * For the hot-tub sketch, which exposes:
 *   - endpoints 0x0A-0x0D: Temperature Measurement (0x0402), one per DS18B20 probe
 *   - endpoints 0x0E-0x10: On/Off (0x0006) for the big pump, circulation pump and heater
 *
 * Creates one child per endpoint, using Hubitat's built-in "Generic Component
 * Temperature Sensor" and "Generic Component Switch" drivers, and routes each
 * endpoint's reports to its child. RELAY_NAMES must list the loads in the same
 * order as relays[] in hot-tub.ino. Child labels are only set when a child is
 * created; use Reset Child Names to apply RELAY_NAMES to existing children.
 */

import hubitat.zigbee.zcl.DataType

metadata {
    definition(name: "ESP32-C6 Hot-Tub controller with DS18B20", namespace: "luisggpina", author: "Luis Pina") {
        capability "Configuration"
        capability "Refresh"
        capability "Sensor"
        command "resetChildNames"
    }
    preferences {
        input name: "logEnable", type: "bool", title: "Enable debug logging", defaultValue: true
    }
}

@groovy.transform.Field static final List<Integer> ENDPOINTS = [0x0A, 0x0B, 0x0C, 0x0D]
@groovy.transform.Field static final List<Integer> RELAY_ENDPOINTS = [0x0E, 0x0F, 0x10, 0x11]
@groovy.transform.Field static final List<String> RELAY_NAMES = ["big pump", "circ pump", "heater", "screen"]

void installed() {
    createChildren()
}

void updated() {
    createChildren()
}

String childDni(Integer ep) {
    return "${device.id}-${ep}"
}

void createChildren() {
    ENDPOINTS.eachWithIndex { Integer ep, int i ->
        String dni = childDni(ep)
        if (!getChildDevice(dni)) {
            String label = "${device.displayName} probe ${i + 1}"
            addChildDevice("hubitat", "Generic Component Temperature Sensor", dni,
                           [name: label, label: label, isComponent: false])
        }
    }
    RELAY_ENDPOINTS.eachWithIndex { Integer ep, int i ->
        String dni = childDni(ep)
        if (!getChildDevice(dni)) {
            String label = "${device.displayName} ${RELAY_NAMES[i]}"
            addChildDevice("hubitat", "Generic Component Switch", dni,
                           [name: label, label: label, isComponent: false])
        }
    }
}

void resetChildNames() {
    RELAY_ENDPOINTS.eachWithIndex { Integer ep, int i ->
        getChildDevice(childDni(ep))?.setLabel("${device.displayName} ${RELAY_NAMES[i]}")
    }
}

List<String> configure() {
    createChildren()
    List<String> cmds = []
    ENDPOINTS.each { Integer ep ->
        String epHex = zigbee.convertToHexString(ep, 2)
        // Bind this endpoint's temperature cluster to the hub
        cmds += "zdo bind 0x${device.deviceNetworkId} 0x${epHex} 0x01 0x0402 {${device.zigbeeId}} {}"
        cmds += "delay 200"
        // Report every 10 s to 10 min, on a change of 0.5 °C (50 = 0.50 in hundredths)
        cmds += zigbee.configureReporting(0x0402, 0x0000, DataType.INT16, 10, 600, 50, [destEndpoint: ep])
    }
    RELAY_ENDPOINTS.each { Integer ep ->
        String epHex = zigbee.convertToHexString(ep, 2)
        // Bind this endpoint's on/off cluster to the hub
        cmds += "zdo bind 0x${device.deviceNetworkId} 0x${epHex} 0x01 0x0006 {${device.zigbeeId}} {}"
        cmds += "delay 200"
        // Report on every change, and at least every 10 min
        cmds += zigbee.configureReporting(0x0006, 0x0000, DataType.BOOLEAN, 0, 600, null, [destEndpoint: ep])
    }
    cmds += refresh()
    return cmds
}

List<String> refresh() {
    List<String> cmds = []
    ENDPOINTS.each { Integer ep ->
        cmds += zigbee.readAttribute(0x0402, 0x0000, [destEndpoint: ep])
    }
    RELAY_ENDPOINTS.each { Integer ep ->
        cmds += zigbee.readAttribute(0x0006, 0x0000, [destEndpoint: ep])
    }
    return cmds
}

Integer childEndpoint(cd) {
    return cd.deviceNetworkId.tokenize('-').last() as Integer
}

void sendZigbee(List<String> cmds) {
    sendHubCommand(new hubitat.device.HubMultiAction(cmds, hubitat.device.Protocol.ZIGBEE))
}

// Called by a child's Refresh button
void componentRefresh(cd) {
    Integer ep = childEndpoint(cd)
    Integer cluster = (ep in RELAY_ENDPOINTS) ? 0x0006 : 0x0402
    sendZigbee(zigbee.readAttribute(cluster, 0x0000, [destEndpoint: ep]))
}

// Called by a relay child's On / Off buttons
void componentOn(cd) {
    relayCommand(childEndpoint(cd), true)
}

void componentOff(cd) {
    relayCommand(childEndpoint(cd), false)
}

void relayCommand(Integer ep, boolean on) {
    String epHex = zigbee.convertToHexString(ep, 2)
    String cmd = on ? "0x01" : "0x00"  // On/Off cluster commands: 0x01 = On, 0x00 = Off
    sendZigbee(["he cmd 0x${device.deviceNetworkId} 0x${epHex} 0x0006 ${cmd} {}",
                "delay 500"] +
               zigbee.readAttribute(0x0006, 0x0000, [destEndpoint: ep]))
}

void parse(String description) {
    Map d = zigbee.parseDescriptionAsMap(description)
    if (logEnable) log.debug "parse: ${d}"

    // On/Off commands the board sends whenever a relay changes (see setRelay() in
    // hot-tub.ino): command 01 = on, 00 = off, from the relay's endpoint.
    if (d?.clusterInt == 0x0006 && d?.isClusterSpecific && d?.command in ["00", "01"]) {
        handleSwitch(Integer.parseInt(d.sourceEndpoint, 16), d.command)
        return
    }

    String epHex = d?.endpoint ?: d?.sourceEndpoint
    if (!epHex || d?.attrInt != 0x0000 || !d?.value) {
        return
    }
    if (d.clusterInt == 0x0402) {
        handleTemperature(Integer.parseInt(epHex, 16), d.value)
    } else if (d.clusterInt == 0x0006) {
        handleSwitch(Integer.parseInt(epHex, 16), d.value)
    }
}

void handleSwitch(Integer ep, String valueHex) {
    def child = getChildDevice(childDni(ep))
    if (!child) {
        log.warn "No child device for endpoint ${ep}"
        return
    }
    String value = (Integer.parseInt(valueHex, 16) != 0) ? "on" : "off"
    child.parse([[name: "switch", value: value,
                  descriptionText: "${child.displayName} was turned ${value}"]])
}

void handleTemperature(Integer ep, String valueHex) {
    def child = getChildDevice(childDni(ep))
    if (!child) {
        log.warn "No child device for endpoint ${ep}"
        return
    }
    Integer raw = hexStrToSignedInt(valueHex)
    if (raw == -32768) {  // 0x8000 = invalid / no reading
        return
    }
    BigDecimal celsius = raw / 100.0
    String scale = location.temperatureScale
    BigDecimal t = (scale == "F") ? celsius * 9 / 5 + 32 : celsius
    t = t.setScale(1, BigDecimal.ROUND_HALF_UP)
    child.parse([[name: "temperature", value: t, unit: "°${scale}",
                  descriptionText: "${child.displayName} temperature is ${t}°${scale}"]])
}
