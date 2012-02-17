#!/usr/bin/env python
import roslib; roslib.load_manifest('automow_node')

import rospy

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from automow_node.msg import Automow_PCB

global diag_publisher

BATTERY_STATES = { 0: "Discharging",
                   1: "Recovery Charging",
                   2: "Charging",
                   3: "Trickle Charging",
                   4: "Critical Discharging",
                   5: "Error" }

def status_cb(msg):
    global BATTERY_STATES
    global diag_publisher
    diag_msg = DiagnosticArray()
    diag_msg.header.stamp = rospy.Time.now()
    diag_msg.status = []
    
    temp_status = DiagnosticStatus()
    temp_status.name = "Chassis Temperature"
    temp_status.hardware_id = "automow_pcb"
    temp_status.values = []
    top_F = msg.temperature_1 * 9/5 + 32
    bot_F = msg.temperature_2 * 9/5 + 32
    temp_status.values.append(KeyValue(key="Top Celsius",
                                       value="%3.2f C"%msg.temperature_1))
    temp_status.values.append(KeyValue(key="Bottom Celsius",
                                       value="%3.2f C"%msg.temperature_2))
    temp_status.values.append(KeyValue(key="Top Fahrenheit",
                                       value="%3.2f F"%(top_F)))
    temp_status.values.append(KeyValue(key="Bottom Fahrenheit",
                                       value="%3.2f F"%(bot_F)))
    if top_F > 100 or bot_F > 100:
        temp_status.message = "High Temperature"
        temp_status.level = temp_status.WARN
    elif top_F > 125 or bot_F > 125:
        temp_status.message = "Critical Temperature"
        temp_status.level = temp_status.ERROR
    else:
        temp_status.message = "OK"
        temp_status.level = temp_status.OK
    diag_msg.status.append(temp_status)

    batt_status = DiagnosticStatus()
    batt_status.name = "Battery Status"
    batt_status.hardware_id = "automow_pcb"
    batt_status.values = []
    state = BATTERY_STATES[msg.battery_state]
    batt_status.values.append(KeyValue(key="State",
                                       value=state))
    batt_status.values.append(KeyValue(key="Charge",
                                       value="{:.0%}".format(msg.charge/100.0)))
    batt_status.values.append(KeyValue(key="Voltage",
                                       value="%3.2f V"%(msg.voltage/1000.0)))
    batt_status.values.append(KeyValue(key="Battery Current",
                                       value="%3.2f A"%(msg.current/1000.0)))
    diag_msg.status.append(batt_status)
    if msg.battery_state >= 4:
        batt_status.level = batt_status.ERROR
    else:
        batt_status.level = batt_status.OK
    batt_status.message = state
    diag_publisher.publish(diag_msg)

def diagnostics_bridge():
    global diag_publisher
    rospy.init_node('diagnostics_bridge', anonymous=True)
    rospy.Subscriber("/automow_pcb/status", Automow_PCB, status_cb)
    diag_publisher = rospy.Publisher("/diagnostics", DiagnosticArray)
    rospy.spin()

if __name__ == '__main__':
    diagnostics_bridge()
