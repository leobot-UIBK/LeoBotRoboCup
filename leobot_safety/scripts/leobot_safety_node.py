#!/usr/bin/env python
# (c) Burkhard Moser burkhard.moser@student.uibk.ac.at
# uses pymodbus
import rospy
import numpy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from pymodbus.client.sync import ModbusTcpClient

#zone 1 = default, zone 2 = further out
zone = 1
zone2speed = 0.02

def getMean(first_digit, second_digit):
    temp =  pow(first_digit, 2) + pow(second_digit, 2)
    return numpy.sqrt(temp)

def getRegisterValue(client, number):
    return client.registers[number]

def getRegisterByte(regvalue, position):
    a = regvalue & (1<<position)
    
    if (a == (1<<position)):
        return True
    else:
        return False

def setRegister(value, position=numpy.NaN):
    #ToDo: send to modbus 
    return

def processSpeed(data):
    
    global zone

    x_speed = round(data.twist.twist.linear.x , 2)
    y_speed = round(data.twist.twist.linear.y , 2)

    if getMean(x_speed, y_speed) >= zone2speed and (zone == 1):
        zone = 2
        setRegister(zone)        
    elif getMean(x_speed, y_speed) <= zone2speed and (zone == 2):
        zone = 1
        setRegister(zone)

def leobot_safety_node():
    
    global zone2speed
    
    pub_fv = rospy.Publisher('/leobot_safety/field_violation', Bool, queue_size=10)
    pub_estop = rospy.Publisher('/leobot_safety/estop', Bool, queue_size=10)
    pub_230 = rospy.Publisher('/leobot_safety/mains_voltage', Bool, queue_size=10)
    pub_charg = rospy.Publisher('/leobot_safety/charging', Bool, queue_size=10)
    pub_zone2 = rospy.Publisher('/leobot_safety/zone', Int16, queue_size=10)

    rospy.Subscriber("/leobot_base/odom", Odometry, processSpeed)
    
    rospy.init_node('leobot_safety_node', anonymous=True)
    
    # publish with 5 Hz
    rate = rospy.Rate(5)
    rospy.loginfo("Starting")
    
    ip = rospy.get_param('/leobot_safety/ip', '192.168.24.25')
    port = rospy.get_param('/leobot_safety/port', '502')
    
    zone2speed = rospy.get_param('/leobot_safety/zone2', '0.02')
    input_addr = rospy.get_param('/leobot_safety/input_addr', '999')
    output_addr = rospy.get_param('/leobot_safety/output_addr', '1999')
    
    rospy.loginfo("Connecting to %s" %ip) 
    client = ModbusTcpClient(ip)
    rospy.loginfo("Connected")
    
    #while True:
    #    status = False
    #    client = ModbusTcpClient(ip)
    #    
    #    status = client.is_socket_open()
    #    
    #    if status:
    #        break
    #    
    #    rospy.sleep(5.)
    #    rospy.loginfo("Can not connect to %s" %ip)

    while not rospy.is_shutdown():
        #read registers
        result = client.read_holding_registers(input_addr,29,unit=1)
        #for i in range(500):
            #print(str(i+1990)) 
            #print(getRegisterByte((getRegisterValue(result, 2)), i))
            #client.write_register(i+1990, 255)
        client.write_register(80, 1)
        #publisher:
        pub_fv.publish(not(getRegisterByte((getRegisterValue(result, 8)), 11))) 
        # is true if the field is violated; motor STO is triggered 
        pub_estop.publish(not(getRegisterByte((getRegisterValue(result, 8)), 10)))
        # estop is true if the relais for the panda disconnected 
        pub_230.publish(getRegisterByte((getRegisterValue(result, 2)), 13)) 
        # is true if the robot is connected to the 230V plug
        pub_charg.publish(not(getRegisterByte((getRegisterValue(result, 2)), 12))) 
        # is true if the plug to charge the battery is connected 
        pub_zone2.publish(zone)
        # is true if in second zone (faster rollin)

        rate.sleep()

    client.close()

if __name__ == '__main__':
    try:
        leobot_safety_node()
    except rospy.ROSInterruptException:
        pass
